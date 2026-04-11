#ifndef ESTIMATOR_H
#define ESTIMATOR_H

#include "LowState.h"
#include "eigen3/Eigen/Dense"
#include "mathtool.h"
#include "FSM/EnumClassList.h"
#include "Kenimatics_normal_solution.h"

class Estimator
{
public:
    Estimator(LowState* lowstate,Eigen::Matrix<int, 4, 1>* contact,Eigen::Matrix<double, 4, 1> * phase,double dt);
    void init();
    Eigen::Matrix<double, 3, 1>  getPosition();
    Eigen::Matrix<double, 3, 1>  getVelocity();
    Eigen::Matrix<double, 3, 1>  getFootPos(int i);
    Eigen::Matrix<double, 3, 4> getFeetPos();
    Eigen::Matrix<double, 3, 4> getFeetVel();
    Eigen::Matrix<double, 3, 4> getPosFeet2BGlobal();
    void run();
private:
    Eigen::Matrix<double, 18, 1>  _xhat;    // 先验x
    Eigen::Matrix<double, 3, 1>   _u;       // 输入变量 (xyz的三轴加速度)            
    Eigen::Matrix<double, 28,  1> _y;       // 输出y
    Eigen::Matrix<double, 28,  1> _yhat;    // 先验y
    Eigen::Matrix<double, 18, 18> _A;       // 状态转移矩阵
    Eigen::Matrix<double, 18, 3>  _B;       // 输入矩阵
    Eigen::Matrix<double, 28, 18> _C;       // 输出矩阵
    /*噪声*/
    Eigen::Matrix<double, 18, 18> _P;       // 预测P协方差矩阵
    Eigen::Matrix<double, 18, 18> _Ppriori; // 先验预测P协方差矩阵
    Eigen::Matrix<double, 18, 18> _Q;       // 过程噪声协方差矩阵
    Eigen::Matrix<double, 28, 28> _R;       // 测量噪声协方差矩阵
    Eigen::Matrix<double, 18, 18> _QInit;   // 过程噪声协方差矩阵 初始化
    Eigen::Matrix<double, 28, 28> _RInit;   // 测量噪声协方差矩阵 初始化
    Eigen::Matrix<double, 18, 1>  _Qdig;    // 过程噪声协方差矩阵 的对角线上的值组成的向量
    Eigen::Matrix<double, 3, 3>   _Cu;      // 输入_u的协方差矩阵

    AvgCov *                      _RCheck;
    AvgCov *                      _uCheck;
    // 输出的测量数据
    Eigen::Matrix<double, 12, 1>  _feetPos2Body;    // 腿相对与机身的位置 - 在{s}
    Eigen::Matrix<double, 12, 1>  _feetVel2Body;    // 腿相对与机身的速度 - 在{s}
    Eigen::Matrix<double,  4, 1>  _feetH;           // 腿相对与机身的高度 - 在{s}
    Eigen::Matrix<double, 28, 28> _S;               // _S = C*P*C.T + R

    Eigen::Matrix<double, 3, 3>   _rotMatB2G;       // Rsb 旋转矩阵
    Eigen::Matrix<float, 3, 1>    _g;                // 重力加速度
    LowState*                     _lowstate;
    Eigen::Matrix<double, 3, 4>   _feetPosGlobalKine, _feetVelGlobalKine;   // 运动学正解所得的 位置 、 速度
    Eigen::Matrix<double, 4, 1> * _phase;           // 步态的频率
    double                        _dt;
    double                        _largeVariance;   // 增幅
    Eigen::Matrix<int, 4, 1>*     _contact      ;   // 判断足端是否接触地面（通过力矩大小）
    double                        _trust;

    Eigen::PartialPivLU<Eigen::Matrix<double, 28, 28>> _Slu;    // _S.lu()
    Eigen::Matrix<double, 28,  1> _Sy;              // _Sy = _S.inv() * (y - yhat)
    Eigen::Matrix<double, 28, 18> _Sc;              // _Sc = _S.inv() * C
    Eigen::Matrix<double, 28, 28> _SR;              // _SR = _S.inv() * R
    Eigen::Matrix<double, 28, 18> _STC;             // _STC = (_S.transpose()).inv() * C
    Eigen::Matrix<double, 18, 18> _IKC;             // _IKC = I - KC
};

Estimator::Estimator(LowState* lowstate,Eigen::Matrix<int, 4, 1>* contact,Eigen::Matrix<double, 4, 1> * phase,double dt):
_lowstate(lowstate),_contact(contact),_phase(phase),_dt(dt){

    for(int i(0); i<_Qdig.rows(); ++i){
        if(i < 3){
            _Qdig(i) = 0.003;
        }
        else if(i < 6){
            _Qdig(i) = 0.003;
        }
        else{
            _Qdig(i) = 0.01;         // 0.00003
        }
    }

    init();
}

void Estimator::init(){
    _g << 0,0,-9.81;
    _largeVariance = 100;

    _xhat.setZero();
    _u.setZero();
    _A.setZero();
    _C.setZero();
    _P.setIdentity();           // 对角线设为1 ，非对角线设为0
    _P = _largeVariance * _P;   // 开机时 使其极度信任测量值

    _A.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    _A.block(0, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * _dt;
    _A.block(3, 3, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
    _A.block(6, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);

    _B.block(3, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3) * _dt;

    _C.block(0, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(3, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(6, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(9, 0, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(12, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(15, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(18, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(21, 3, 3, 3) = -Eigen::MatrixXd::Identity(3, 3);
    _C.block(0, 6, 12, 12) = Eigen::MatrixXd::Identity(12, 12);
    _C(24, 8) = 1;
    _C(25, 11) = 1;
    _C(26, 14) = 1;
    _C(27, 17) = 1;

    _RInit<<
               0.008 , 0.012 ,-0.000 ,-0.009 , 0.012 , 0.000 , 0.009 ,-0.009 ,-0.000 ,-0.009 ,-0.009 , 0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.001 ,-0.002 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               0.012 , 0.019 ,-0.001 ,-0.014 , 0.018 ,-0.000 , 0.014 ,-0.013 ,-0.000 ,-0.014 ,-0.014 , 0.001 ,-0.001 , 0.001 ,-0.001 , 0.000 , 0.000 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.001,  0.001, -0.001,  0.000, -0.000,  0.000, -0.000,  0.001,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000,  0.001, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               0.012 , 0.018 ,-0.001 ,-0.013 , 0.018 ,-0.000 , 0.013 ,-0.013 ,-0.000 ,-0.013 ,-0.013 , 0.001 ,-0.001 , 0.000 ,-0.001 , 0.000 , 0.001 ,-0.001 ,-0.003 , 0.000 ,-0.001 ,-0.004 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.001 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
               0.009 , 0.014 ,-0.000 ,-0.010 , 0.013 , 0.000 , 0.010 ,-0.010 ,-0.000 ,-0.010 ,-0.010 , 0.000 ,-0.001 , 0.000 ,-0.001 , 0.000 ,-0.000 ,-0.001 ,-0.001 , 0.000 ,-0.000 ,-0.003 ,-0.000 ,-0.001 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.009, -0.013,  0.000,  0.010, -0.013,  0.000, -0.010,  0.009,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002,  0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               -0.000, -0.000, -0.000,  0.000, -0.000, -0.000, -0.000,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.000, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.001,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001,  0.000,  0.000, -0.000, -0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.000,  0.001,  0.000,  0.000,  0.000,  0.000,
               -0.009, -0.014,  0.000,  0.010, -0.013,  0.000, -0.010,  0.010,  0.000,  0.010,  0.010, -0.000,  0.001, -0.000,  0.000, -0.000,  0.000,  0.001,  0.002, -0.000,  0.000,  0.003,  0.001,  0.001,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.001 ,-0.000 ,-0.000 , 0.001 ,-0.000 , 0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.001 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.000,  0.001, -0.001, -0.000, -0.001,  0.001,  0.000,  0.001,  0.001,  0.000,  1.708,  0.048,  0.784,  0.062,  0.042,  0.053,  0.077,  0.001, -0.061,  0.046, -0.019, -0.029,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.001 ,-0.000 , 0.000 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 ,-0.000 , 0.048 , 5.001 ,-1.631 ,-0.036 , 0.144 , 0.040 , 0.036 , 0.016 ,-0.051 ,-0.067 ,-0.024 ,-0.005 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001,  0.000,  0.000, -0.001, -0.000, -0.001,  0.000,  0.000,  0.000,  0.000, -0.000,  0.784, -1.631,  1.242,  0.057, -0.037,  0.018,  0.034, -0.017, -0.015,  0.058, -0.021, -0.029,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 , 0.000 , 0.001 , 0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 ,-0.000 , 0.062 ,-0.036 , 0.057 , 6.228 ,-0.014 , 0.932 , 0.059 , 0.053 ,-0.069 , 0.148 , 0.015 ,-0.031 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000,  0.000, -0.000, -0.000,  0.001,  0.000, -0.000,  0.000,  0.000, -0.000,  0.000,  0.000,  0.042,  0.144, -0.037, -0.014,  3.011,  0.986,  0.076,  0.030, -0.052, -0.027,  0.057,  0.051,  0.000,  0.000,  0.000,  0.000,
               -0.001, -0.001, -0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000,  0.053,  0.040,  0.018,  0.932,  0.986,  0.885,  0.090,  0.044, -0.055,  0.057,  0.051, -0.003,  0.000,  0.000,  0.000,  0.000,
               -0.002, -0.003,  0.000,  0.002, -0.003, -0.000, -0.001,  0.002,  0.000,  0.002,  0.002, -0.000,  0.077,  0.036,  0.034,  0.059,  0.076,  0.090,  6.230,  0.139,  0.763,  0.013, -0.019, -0.024,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 ,-0.000 ,-0.000 , 0.000 ,-0.000 , 0.000 , 0.000 ,-0.000 ,-0.000 ,-0.000 , 0.000 , 0.001 , 0.016 ,-0.017 , 0.053 , 0.030 , 0.044 , 0.139 , 3.130 ,-1.128 ,-0.010 , 0.131 , 0.018 , 0.000 , 0.000 , 0.000 , 0.000,
               -0.000, -0.001, -0.000,  0.000, -0.001, -0.000, -0.000,  0.000,  0.000,  0.000,  0.000,  0.000, -0.061, -0.051, -0.015, -0.069, -0.052, -0.055,  0.763, -1.128,  0.866, -0.022, -0.053,  0.007,  0.000,  0.000,  0.000,  0.000,
               -0.003, -0.004, -0.000,  0.003, -0.004, -0.000, -0.003,  0.003,  0.000,  0.003,  0.003,  0.000,  0.046, -0.067,  0.058,  0.148, -0.027,  0.057,  0.013, -0.010, -0.022,  2.437, -0.102,  0.938,  0.000,  0.000,  0.000,  0.000,
               -0.000, -0.000,  0.000,  0.000, -0.000,  0.000, -0.000,  0.000, -0.000,  0.000,  0.001,  0.000, -0.019, -0.024, -0.021,  0.015,  0.057,  0.051, -0.019,  0.131, -0.053, -0.102,  4.944,  1.724,  0.000,  0.000,  0.000,  0.000,
               -0.001, -0.001,  0.000,  0.001, -0.001,  0.000, -0.001,  0.001, -0.000,  0.001,  0.001,  0.000, -0.029, -0.005, -0.029, -0.031,  0.051, -0.003, -0.024,  0.018,  0.007,  0.938,  1.724,  1.569,  0.000,  0.000,  0.000,  0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0 , 0.000,
               0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 0.000 , 1.0;

    _Cu << 268.573,  -43.819, -147.211,
            -43.819 ,  92.949 ,  58.082,
            -147.211,   58.082,  302.120; 

    _QInit = _Qdig.asDiagonal();
    _QInit +=  _B * _Cu * _B.transpose();

    // _RCheck  = new AvgCov(28,  " R");
    // _uCheck  = new AvgCov(3,   " u");
}

void Estimator::run(){
    
    _feetH.setZero();
    _feetPosGlobalKine = GetFeetPos2BODY(*_lowstate,FrameType::GLOBAL);
    _feetVelGlobalKine = GetFeetSpeed2BODY(*_lowstate,FrameType::GLOBAL);
    _Q = _QInit;    // 必须赋值，后面根据步态进行调整
    _R = _RInit;
    
    for(int i(0);i<4;++i){        
        
        if((*_contact)(i) == 0){
            // 使足端i位置不可信
            _Q.block(6+3*i, 6+3*i, 3, 3) = _largeVariance * Eigen::MatrixXd::Identity(3, 3);
            // 足端i速度不可信
            _R.block(12+3*i, 12+3*i, 3, 3) = _largeVariance * Eigen::MatrixXd::Identity(3, 3);
            // 足端i的高度不可信
            _R(24+i, 24+i) = _largeVariance;
        }   
        else{                                                               //_phase：4 只脚的步态相位  
            _trust = windowFunc((*_phase)(i), 0.2);
            _Q.block(6+3*i, 6+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _QInit.block(6+3*i, 6+3*i, 3, 3);
            _R.block(12+3*i, 12+3*i, 3, 3) = (1 + (1-_trust)*_largeVariance) * _RInit.block(12+3*i, 12+3*i, 3, 3);
            _R(24+i, 24+i) = (1 + (1-_trust)*_largeVariance) * _RInit(24+i, 24+i);
        }
        _feetPos2Body.segment(3*i,3) = _feetPosGlobalKine.col(i);
        _feetVel2Body.segment(3*i,3) = _feetVelGlobalKine.col(i);
    }

    _rotMatB2G = _lowstate->_imu.GetRotMat().cast<double>();
    Eigen::Matrix<float,3,1>e = Quat2Euler(_lowstate->_imu.GetQuat());
    _u = _rotMatB2G * _lowstate->_imu.GetAcc().cast<double>() + _g.cast<double>();
   
    _xhat = _A * _xhat + _B * _u;
    _yhat = _C * _xhat;
    _y << _feetPos2Body,_feetVel2Body,_feetH;

    _Ppriori = _A * _P * _A.transpose() + _Q;
    _S =  _R + _C * _Ppriori * _C.transpose();
    _Slu = _S.lu();             
    _Sy = _Slu.solve(_y - _yhat);
    _Sc = _Slu.solve(_C);
    _SR = _Slu.solve(_R);
    _STC = (_S.transpose()).lu().solve(_C);
    _IKC = Eigen::MatrixXd::Identity(18, 18) - _Ppriori*_C.transpose()*_Sc;

    _xhat += _Ppriori * _C.transpose() * _Sy;
    _P =  _IKC * _Ppriori * _IKC.transpose()
        + _Ppriori * _C.transpose() * _SR * _STC * _Ppriori.transpose();

    // printf(" x:%.4f y:%.4f z:%.4f vx:%.4f vy:%.4f vz:%.4f fx:%.4f fy:%.4f fz:%.4f\n",_xhat(0),_xhat(1),_xhat(2),_xhat(3),_xhat(4),_xhat(5),_xhat(6),_xhat(7),_xhat(8));
    // printf(" x:%.4f y:%.4f z:%.4f  fx:%.4f fy:%.4f fz:%.4f\n",_xhat(0),_xhat(1),_xhat(2),_xhat(6),_xhat(7),_xhat(8));
    printf(" x:%.4f y:%.4f z:%.4f  vx:%.4f vy:%.4f vz:%.4f\n",_xhat(0),_xhat(1),_xhat(2),_xhat(3),_xhat(4),_xhat(5));

    // std::cout<<"-----"<< _feetPos2Body <<"----"<< std::endl;
    // std::cout<<"_rotMatB2G:"<<rotMatToRPY(_rotMatB2G.cast<float>())/M_PI*180 << std::endl;
    // std::cout<<"****"<< _rotMatB2G <<"----"<< std::endl;
    // printf("accx:%.5f accy:%.5f accz:%.5f\n",_lowstate->_imu.GetAcc()(0),_lowstate->_imu.GetAcc()(1),_lowstate->_imu.GetAcc()(2));
    // std::cout<<"u:"<< _u << std::endl;
    // printf("fx0:%.4f fy0:%.4f fz0:%.4f\n",_xhat(6),_xhat(7),_xhat(8));
    // printf("fx1:%.4f fy1:%.4f fz1:%.4f\n",_xhat(9),_xhat(10),_xhat(11));
    // printf("fx2:%.4f fy2:%.4f fz2:%.4f\n",_xhat(12),_xhat(13),_xhat(14));
    // printf("fx3:%.4f fy3:%.4f fz3:%.4f\n",_xhat(15),_xhat(16),_xhat(17));
    // _uCheck->Measure(_u);
    // _RCheck->Measure(_y);

}

Eigen::Matrix<double, 3, 1>  Estimator::getPosition(){
    return _xhat.segment(0, 3);
}
Eigen::Matrix<double, 3, 1>  Estimator::getVelocity(){
    // Eigen::Matrix<double, 3, 1> v;
    // v<< 0.07,0,0;
    return _xhat.segment(3, 3);
    // return v;
}
Eigen::Matrix<double, 3, 1>  Estimator::getFootPos(int i){
    return getPosition() + _lowstate->_imu.GetRotMat().cast<double>() * GetFeetPos2BODY(*_lowstate, i, FrameType::BODY);
}
Eigen::Matrix<double, 3, 4> Estimator::getFeetPos(){
    Eigen::Matrix<double, 3, 4> feetPos;
    for(int i(0); i < 4; ++i){
        feetPos.col(i) = getFootPos(i);
    }
    return feetPos;
}
Eigen::Matrix<double, 3, 4> Estimator::getFeetVel(){
    Eigen::Matrix<double, 3, 4> feetVel = GetFeetSpeed2BODY(*_lowstate, FrameType::GLOBAL);
    for(int i(0); i < 4; ++i){
        feetVel.col(i) += getVelocity();
    }
    return feetVel;
}
Eigen::Matrix<double, 3, 4> Estimator::getPosFeet2BGlobal(){
    Eigen::Matrix<double, 3, 4> feet2BPos;
    for(int i(0); i < 4; ++i){
         feet2BPos.col(i) = getFootPos(i) - getPosition();
    }
    return feet2BPos;
}


#endif