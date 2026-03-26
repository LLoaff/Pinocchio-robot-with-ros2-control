#ifndef BALANCECTRL_H
#define BALANCECTRL_H

#include <eigen3/Eigen/Dense>
#include <QuadProg++/QuadProg++.hh>
#include "mathtool.h"

class BalanceCtrl
{
public:
    BalanceCtrl();
    Eigen::Matrix<double, 3, 4> calF(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, 
    Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<int, 4, 1> contact);

private:

    void calMatrixA(Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<int, 4, 1> contact);// 计算动力学方程左侧
    void calVectorBd(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, Eigen::Matrix<double, 3, 3> rotM);//计算动力学方程bd
    void calConstraints(Eigen::Matrix<int, 4, 1> contact);// 化整为求解器格式
    void solveQP();// 调用求解器
    Eigen::Matrix<double, 12, 12> _G, _W, _U;   // _G解算器     _W足端力大小权重    _U足端力改变量权重
    Eigen::Matrix<double, 6, 6> _S;            // 动力学方程权重
    Eigen::Matrix<double, 3, 3> _Ib;           // 惯性张量
    Eigen::Matrix<double, 6, 1> _bd;           // 动力学方程右侧矩阵
    Eigen::Matrix<double, 3, 1> _g;        // 重力加速度
    Eigen::Matrix<double, 3, 1> _pcb;      // 质心在机身坐标系下的位置向量
    Eigen::Matrix<double, 12, 1> _F, _Fprev, _g0T; // _g0T ： _g0的转置
    double _mass, _alpha, _beta, _fricRatio;    // _mass质量   _alpha_W权重系数   _beta _U权重系数 _fricRatio摩擦系数
    Eigen::MatrixXd _CE, _CI;
    Eigen::VectorXd _ce0, _ci0;
    Eigen::Matrix<double, 6 , 12> _A;           // 动力学方程左侧矩阵
    Eigen::Matrix<double, 5 , 3 > _fricMat;     // 摩擦四棱柱约束矩阵

    quadprogpp::Matrix<double> G, CE, CI;
    quadprogpp::Vector<double> g0, ce0, ci0, x;
};

BalanceCtrl::BalanceCtrl(){
    Eigen::Matrix<double, 6 , 1> s;
    Eigen::Matrix<double, 12 , 1> w,u;
    _mass = 12.7;
    _pcb<< 0,0,0;
    _Ib<< 0.542828,0.141737,0.170247,
          0.141737,0.496037,0.228799,
          0.170247,0.228799,0.498639;
    // _Ib<<0.1051041666666,0,        0,
    //      0,              0.3271875,0,
    //      0,              0,        0.3854166666666;
    _g << 0, 0, -9.81;

    w << 100, 100, 60, 100, 100, 60, 100, 100, 60, 100, 100, 60;

    u << 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10, 10;
    s << 20, 20, 60, 550, 550, 550;

    _alpha = 3.8;
    _beta  = 0.1;
    _fricRatio = 0.4;
    _S = s.asDiagonal();
    _W = w.asDiagonal();
    _U = u.asDiagonal();
    
    _Fprev.setZero();
    _fricMat <<  1,  0, _fricRatio,
                -1,  0, _fricRatio,
                 0,  1, _fricRatio,
                 0, -1, _fricRatio,
                 0,  0, 1;
}

Eigen::Matrix<double, 3, 4> BalanceCtrl:: calF(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, 
Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<int, 4, 1> contact){ 
    // for(int i(0);i<4;++i){
    //     printf("id:%d x:%.3f y:%.3f z:%.3f \n",i,feetPos2B.col(i)(0),feetPos2B.col(i)(1),feetPos2B.col(i)(2));   
    // }
    calMatrixA(feetPos2B, rotM, contact);
    calVectorBd(ddPcd, dWbd, rotM);
    calConstraints(contact);

    _G = _A.transpose()*_S*_A + _alpha*_W + _beta*_U;
    // Eigen::Matrix<double, 1, 12> _g0;
    // _g0 = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;
    // _g0T = _g0.transpose();
    _g0T = -_bd.transpose()*_S*_A - _beta*_Fprev.transpose()*_U;
    solveQP();
    _Fprev = _F;
    // std::cout<<"F:\n"<<_Fprev<<" --- \n"<<std::endl;
    return vec12ToVec34(_F);
}

void BalanceCtrl::calMatrixA(Eigen::Matrix<double, 3, 4> feetPos2B, Eigen::Matrix<double, 3, 3> rotM, Eigen::Matrix<int, 4, 1> contact){
    for(int i(0); i < 4; ++i){
        _A.block(0, 3*i, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
        _A.block(3, 3*i, 3, 3) = skew(feetPos2B.col(i) - rotM*_pcb);    // skew 生成反对称矩阵，用于叉乘
    }
}   

void BalanceCtrl::calVectorBd(Eigen::Matrix<double, 3, 1> ddPcd, Eigen::Matrix<double, 3, 1> dWbd, Eigen::Matrix<double, 3, 3> rotM){
    _bd.head(3) = _mass * (ddPcd - _g);
    _bd.tail(3) = (rotM * _Ib * rotM.transpose()) * dWbd;
}

void BalanceCtrl::calConstraints(Eigen::Matrix<int, 4, 1> contact){
    int contactLegNum = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            contactLegNum += 1;
        }
    }

    _CI.resize(5*contactLegNum, 12);
    _ci0.resize(5*contactLegNum);
    _CE.resize(3*(4 - contactLegNum), 12);
    _ce0.resize(3*(4 - contactLegNum));

    _CI.setZero();
    _ci0.setZero();
    _CE.setZero();
    _ce0.setZero();

    int ceID = 0;
    int ciID = 0;
    for(int i(0); i<4; ++i){
        if(contact(i) == 1){
            _CI.block(5*ciID, 3*i, 5, 3) = _fricMat;
            ++ciID;
        }else{
            _CE.block(3*ceID, 3*i, 3, 3) =Eigen::MatrixXd::Identity(3, 3);
            ++ceID;
        }
    }
}

void BalanceCtrl::solveQP(){
    int n = _F.size();
    int m = _ce0.size();
    int p = _ci0.size();

    G.resize(n, n);
    CE.resize(n, m);
    CI.resize(n, p);
    g0.resize(n);
    ce0.resize(m);
    ci0.resize(p);
    x.resize(n);

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < n; ++j) {
            G[i][j] = _G(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < m; ++j) {
            CE[i][j] = (_CE.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        for (int j = 0; j < p; ++j) {
            CI[i][j] = (_CI.transpose())(i, j);
        }
    }

    for (int i = 0; i < n; ++i) {
        g0[i] = _g0T[i];
    }

    for (int i = 0; i < m; ++i) {
        ce0[i] = _ce0[i];
    }

    for (int i = 0; i < p; ++i) {
        ci0[i] = _ci0[i];
    }

    double value = solve_quadprog(G, g0, CE, ce0, CI, ci0, x);

    for (int i = 0; i < n; ++i) {
        _F[i] = x[i];
    }
}
#endif



// #ifndef BALANCECTRL_H
// #define BALANCECTRL_H

// #include <eigen3/Eigen/Dense>
// #include "mathtool.h"
// #include "LowState.h"
// #include "Reversal_solution.h"
// #include "Kenimatics_normal_solution.h"

// class BalanceCtrl
// {
// public:
//     BalanceCtrl();
//     Eigen::Matrix<float, 12, 1> calTau( LowState *_lowstate , Eigen::Matrix<float,12,1> target_pos, Eigen::Matrix<float,12,1> target_speed ,
//        const Eigen::Matrix<int, 4, 1> & contact);

//     Eigen::Matrix<float, 12, 1> calQ( LowState *_lowstate,const Eigen::Matrix<int, 4, 1> & contact);
// private:
//     Eigen::Matrix<float, 3, 1> calOneTau(int legid , Eigen::Matrix<float, 3, 1> q,Eigen::Matrix<float, 3, 1> w , Eigen::Matrix<float,3,1> target_pos,
//          Eigen::Matrix<float,3,1> target_speed ,const Eigen::Matrix<int, 4, 1> & contact);

//     Eigen::Matrix<float,3,3>    Swing_KP;
//     Eigen::Matrix<float,3,3>    Swing_KD;
//     Eigen::Matrix<float,3,3>    Stance_KP;
//     Eigen::Matrix<float,3,3>    Stance_KD;
// };

// BalanceCtrl::BalanceCtrl(){
//     Stance_KP<< 2.0,  0 ,   0,
//                 0 ,   2.0,  0,
//                 0 ,   0 ,   4.5;

//     Stance_KD<< 0.5, 0,    0,
//                 0,    0.5, 0,
//                 0,    0,    0.5;


// }

// Eigen::Matrix<float, 12, 1> BalanceCtrl::calTau(  LowState * _lowstate , Eigen::Matrix<float,12,1> target_pos, 
// Eigen::Matrix<float,12,1> target_speed ,const Eigen::Matrix<int, 4, 1> & contact){ 
//     Eigen::Matrix<float, 12, 1> motor_q ;
//     Eigen::Matrix<float, 12, 1> motor_w ;
//     Eigen::Matrix<float, 12, 1> tau;
//     for(int i(0);i<12;i++){
//         motor_q(i) = _lowstate->Motor_Angle[i];
//         motor_w(i) = _lowstate->_motor_data[i].dq;
//     }
    
//     for(int legid(0);legid<4;++legid){
//         tau.segment(3*legid,3) = calOneTau(legid,motor_q.segment(3*legid,3),motor_w.segment(3*legid,3),
//             target_pos.segment(3*legid,3),target_speed.segment(3*legid,3),contact);
//     }
//     // std::cout<< " contact:"<<  contact<<std::endl;
//     return tau;
// }

// Eigen::Matrix<float, 12, 1> BalanceCtrl::calQ( LowState * _lowstate,const Eigen::Matrix<int, 4, 1> & contact){
//     Eigen::Matrix<float, 3, 3> rotmat;
//     Eigen::Matrix<float, 12, 1> q;
//     Eigen::Matrix<float, 3, 4> pos;
//     rotmat = BalanceRPY(_lowstate->_imu.GetQuat());

//     pos = rotmat * GetFeetPos2BODY(*_lowstate,FrameType::BODY).cast<float>();
//     for(int i(0);i<4;++i){
//         q.segment(3*i,3) = Reversal_Update_B(i,pos.col(i)(0),pos.col(i)(1),pos.col(i)(2));
//     }
//     return q; 
// }

// Eigen::Matrix<float, 3, 1> BalanceCtrl::calOneTau(int legid , Eigen::Matrix<float, 3, 1> q ,Eigen::Matrix<float, 3, 1> w,
// Eigen::Matrix<float,3,1> target_pos,Eigen::Matrix<float,3,1> target_speed ,const Eigen::Matrix<int, 4, 1> & contact){
//     Eigen::Matrix<float,3,1> f; 
//     Eigen::Matrix<float,3,1> tau;
//     Eigen::Matrix<float,3,1> pos_xyz; // 存储xyz
//     Eigen::Matrix<float,3,1> pos_s;   // 存储xyz的速度
//     Eigen::Matrix<float,3,3> Jac;
//     float l1 = _labad_;
//     float l2 = -_lhip_;
//     float l3 = -_lknee_;
    
//     if(legid == 0 || legid == 2)
//         l1 = -_labad_;
//     // 计算雅可比 position
//     float s1 = std::sin(q(0));
//     float s2 = std::sin(q(1));
//     float s3 = std::sin(q(2));

//     float c1 = std::cos(q(0));
//     float c2 = std::cos(q(1));
//     float c3 = std::cos(q(2));

//     float c23 = c2 * c3 - s2 * s3;
//     float s23 = s2 * c3 + c2 * s3;
//     Jac(0, 0) = 0;
//     Jac(1, 0) = -l3 * c1 * c23 - l2 * c1 * c2 - l1 * s1;
//     Jac(2, 0) = -l3 * s1 * c23 - l2 * c2 * s1 + l1 * c1;
//     Jac(0, 1) = l3 * c23 + l2 * c2;
//     Jac(1, 1) = l3 * s1 * s23 + l2 * s1 * s2;
//     Jac(2, 1) = -l3 * c1 * s23 - l2 * c1 * s2;
//     Jac(0, 2) = l3 * c23;
//     Jac(1, 2) = l3 * s1 * s23;
//     Jac(2, 2) = -l3 * c1 * s23;
//     pos_s = Jac * w;    // 三维速度
//     pos_xyz = GetPos_B(legid,q(0),q(1),q(2));
//      // 判断是否是支撑腿
//     if(contact(legid) == 1){
//         f= Stance_KP*(target_pos-pos_xyz)+Stance_KD*(target_speed-pos_s);
//     }else{
//         f= Swing_KP*(target_pos-pos_xyz)+Swing_KD*(target_speed-pos_s);
//     }

//     tau = Jac.transpose()*f;
//     return tau;
// }

// #endif
