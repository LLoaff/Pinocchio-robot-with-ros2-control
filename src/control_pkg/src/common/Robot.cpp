#include "Robot.h"

QuadrupedRobot::QuadrupedRobot(){
    _feetPosNormalStand << 0.26,  0.26,  -0.35,  -0.35,
                          -0.226,  0.226,  -0.226,  0.226,
                          -0.365,-0.365,-0.365,-0.365;
    _robVelLimitX<<-0.8,0.8;
    _robVelLimitY<<-0.5,0.5;
    _robVelLimitYaw<< - 1.5,1.5; // 20度

}


