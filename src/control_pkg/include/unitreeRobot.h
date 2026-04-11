#ifndef UNITREEROBOT_H
#define UNITREEROBOT_H

#include "mathTypes.h"

class QuadrupedRobot{
public:
    QuadrupedRobot();

    Vec34 getFeetPosIdeal(){return _feetPosNormalStand;}
    Vec2 getRobVelLimitX(){return _robVelLimitX;}
    Vec2 getRobVelLimitY(){return _robVelLimitY;}
    Vec2 getRobVelLimitYaw(){return _robVelLimitYaw;}
protected:
    Vec34 _feetPosNormalStand;
    Vec2 _robVelLimitX;
    Vec2 _robVelLimitY;
    Vec2 _robVelLimitYaw;
};

QuadrupedRobot::QuadrupedRobot(){
    _feetPosNormalStand << 0.26,  0.26,  -0.3,  -0.3,
                          -0.146,  0.146,  -0.146,  0.146,
                          -0.365,-0.365,-0.365,-0.365;
    _robVelLimitX<<-0.3,0.3;
    _robVelLimitY<<-0.1,0.1;
    _robVelLimitYaw<< - 0.08726,0.08726; // 5度
}


#endif  