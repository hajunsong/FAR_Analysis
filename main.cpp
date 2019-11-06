#include "RobotArm/robotarm.h"

int main()
{
    RobotArm robot(6,6);

//    robot.run_kinematics();
//    robot.run_inverse_kinematics();
    robot.run_dynamics();

    return 0;
}
