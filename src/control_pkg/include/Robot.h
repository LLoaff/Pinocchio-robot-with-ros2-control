#ifndef ROBOT_H
#define ROBOT_H

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
#endif  