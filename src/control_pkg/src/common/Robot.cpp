#include "Robot.h"

QuadrupedRobot::QuadrupedRobot(){
    _feetPosNormalStand << 0.26,  0.26,  -0.3,  -0.3,
                          -0.146,  0.146,  -0.146,  0.146,
                          -0.365,-0.365,-0.365,-0.365;
    _robVelLimitX<<-0.3,0.3;
    _robVelLimitY<<-0.15,0.15;
    _robVelLimitYaw<< - 0.08726,0.08726; // 5度
}


