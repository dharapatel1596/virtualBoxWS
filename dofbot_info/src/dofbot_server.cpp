#include <iostream>
#include "ros/ros.h"
#include <dofbot_info/kinemarics.h>
#include "dofbot_kinemarics.h"

using namespace KDL;
using namespace std;
Dofbot dofbot = Dofbot();
// Radian to angle
const float RA2DE = 180.0f / M_PI;
// angle to radian
const float DE2RA = M_PI / 180.0f;
const char *urdf_file = "/home/dhara/dofbot_ws/src/dofbot_info/urdf/dofbot.urdf";
int a = 0;


bool srvicecallback(dofbot_info::kinemaricsRequest &request, dofbot_info::kinemaricsResponse &response) {
    if (request.kin_name == "fk") {
        double joints[]{request.cur_joint1, request.cur_joint2, request.cur_joint3, request.cur_joint4,
                        request.cur_joint5};
        // Define the target joint angle package
        vector<double> initjoints;
        // Defining position tolerances
        vector<double> initpos;
        // Target joint angle unit conversion, from angle to radian
        for (int i = 0; i < 5; ++i) initjoints.push_back((joints[i] - 90) * DE2RA);
        // get the current position
        dofbot.dofbot_getFK(urdf_file, initjoints, initpos);
        cout << "--------- Fk ---------" << a << "--------- Fk ---------" << endl;
        cout << "XYZCoordinates ： " << initpos.at(0) << " ," << initpos.at(1) << " ," << initpos.at(2) << endl;
        cout << "Roll,Pitch,Yaw： " << initpos.at(3) << " ," << initpos.at(4) << " ," << initpos.at(5) << endl;
        response.x = initpos.at(0);
        response.y = initpos.at(1);
        response.z = initpos.at(2);
        response.Roll = initpos.at(3);
        response.Pitch = initpos.at(4);
        response.Yaw = initpos.at(5);
    }
    if (request.kin_name == "ik") {
        // Gripping length
        double tool_param = 0.12;
        // Grasping position
        double Roll = 2.5*request.tar_y*100-207.5;
        double Pitch = 0;
        double Yaw = 0;
        // Find the offset angle
        double init_angle = atan2(double(request.tar_x), double(request.tar_y));
        // Find the length of the projection of the gripper on the sloping edge
        double dist = tool_param * sin((180 + Roll) * DE2RA);
        // Find the length of the hypotenuse
        double distance = hypot(request.tar_x, request.tar_y) - dist;
        //Find end position (except gripper)
        double x = distance * sin(init_angle);
        double y = distance * cos(init_angle);
        double z = tool_param * cos((180 + Roll) * DE2RA);
        ///////////////  前后跟随  ///////////////
        if (request.tar_z >= 0.2) {
            x=request.tar_x;
            y=request.tar_y;
            z=request.tar_z;
            Roll= -90;
        }
        // End position (unit: m)
        double xyz[]{x, y, z};
        // End Posture (unit: radian)
        double rpy[]{Roll * DE2RA, Pitch * DE2RA, Yaw * DE2RA};
        cout << xyz[0] << " , " << xyz[1] << " , " << xyz[2] << "\t"
        << rpy[0] << " , " << rpy[1] << " , " << rpy[2] << endl;
        // Create output angle
        vector<double> outjoints;
        // Create end pose 
        vector<double> targetXYZ;
        // Create end pose packet
        vector<double> targetRPY;
        for (int k = 0; k < 3; ++k) targetXYZ.push_back(xyz[k]);
        for (int l = 0; l < 3; ++l) targetRPY.push_back(rpy[l]);
        // Inverse the solution to find the angle of each joint to reach the target point
        dofbot.dofbot_getIK(urdf_file, targetXYZ, targetRPY, outjoints);
        // Print inverse kinematics results
        cout << "--------- Ik ---------" << a << "--------- Ik ---------" << endl;
        for (int i = 0; i < 5; i++) cout << (outjoints.at(i) * RA2DE) + 90 << ",";
        cout << endl;
        a++;
        response.joint1 = (outjoints.at(0) * RA2DE) + 90;
        response.joint2 = (outjoints.at(1) * RA2DE) + 90;
        response.joint3 = (outjoints.at(2) * RA2DE) + 90;
        response.joint4 = (outjoints.at(3) * RA2DE) + 90;
        response.joint5 = (outjoints.at(4) * RA2DE) + 90;
    }
    return true;
}

/*
* This is the ROS   server for the pros and cons of the robotic arm
 * Note: The end mentioned refers to the 5th   steering gear rotation center,
*/
int main(int argc, char **argv) {
    cout << " Waiting to receive ******" << endl;
    //ROS node
    ros::init(argc, argv, "dofbot_server");
    // Create node handle
    ros::NodeHandle n;
    // Create server
    ros::ServiceServer server = n.advertiseService("dofbot_kinemarics", srvicecallback);
    // block
    ros::spin();
    return 0;
}
