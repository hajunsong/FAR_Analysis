#include "RobotArm/robotarm.h"

int main()
{
    RobotArm robot(6,6);

//    robot.run_kinematics();
//    robot.run_inverse_kinematics();
//    robot.run_dynamics();
    robot.run_inverse_kinematics_with_path_generator();

    return 0;
}
