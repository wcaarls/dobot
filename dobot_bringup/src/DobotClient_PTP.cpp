#include "ros/ros.h"
#include "std_msgs/String.h"
#include "dobot_msgs/SetCmdTimeout.h"
#include "dobot_msgs/SetQueuedCmdClear.h"
#include "dobot_msgs/SetQueuedCmdStartExec.h"
#include "dobot_msgs/SetQueuedCmdForceStopExec.h"
#include "dobot_msgs/GetDeviceVersion.h"

#include "dobot_msgs/SetEndEffectorParams.h"
#include "dobot_msgs/SetPTPJointParams.h"
#include "dobot_msgs/SetPTPCoordinateParams.h"
#include "dobot_msgs/SetPTPJumpParams.h"
#include "dobot_msgs/SetPTPCommonParams.h"
#include "dobot_msgs/SetPTPCmd.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DobotClient");
    ros::NodeHandle n;

    ros::ServiceClient client;

    // SetCmdTimeout
    client = n.serviceClient<dobot_msgs::SetCmdTimeout>("/DobotServer/SetCmdTimeout");
    dobot_msgs::SetCmdTimeout srv1;
    srv1.request.timeout = 3000;
    if (client.call(srv1) == false) {
        ROS_ERROR("Failed to call SetCmdTimeout. Maybe DobotServer isn't started yet!");
        return -1;
    }

    // Clear the command queue
    client = n.serviceClient<dobot_msgs::SetQueuedCmdClear>("/DobotServer/SetQueuedCmdClear");
    dobot_msgs::SetQueuedCmdClear srv2;
    client.call(srv2);

    // Start running the command queue
    client = n.serviceClient<dobot_msgs::SetQueuedCmdStartExec>("/DobotServer/SetQueuedCmdStartExec");
    dobot_msgs::SetQueuedCmdStartExec srv3;
    client.call(srv3);

    // Get device version information
    client = n.serviceClient<dobot_msgs::GetDeviceVersion>("/DobotServer/GetDeviceVersion");
    dobot_msgs::GetDeviceVersion srv4;
    client.call(srv4);
    if (srv4.response.result == 0) {
        ROS_INFO("Device version:%d.%d.%d", srv4.response.majorVersion, srv4.response.minorVersion, srv4.response.revision);
    } else {
        ROS_ERROR("Failed to get device version information!");
    }

    // Set end effector parameters
    client = n.serviceClient<dobot_msgs::SetEndEffectorParams>("/DobotServer/SetEndEffectorParams");
    dobot_msgs::SetEndEffectorParams srv5;
    srv5.request.xBias = 70;
    srv5.request.yBias = 0;
    srv5.request.zBias = 0;
    client.call(srv5);

    // Set PTP joint parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPJointParams>("/DobotServer/SetPTPJointParams");
        dobot_msgs::SetPTPJointParams srv;

        for (int i = 0; i < 4; i++) {
            srv.request.velocity.push_back(100);
        }
        for (int i = 0; i < 4; i++) {
            srv.request.acceleration.push_back(100);
        }
        client.call(srv);
    } while (0);

    // Set PTP coordinate parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPCoordinateParams>("/DobotServer/SetPTPCoordinateParams");
        dobot_msgs::SetPTPCoordinateParams srv;

        srv.request.xyzVelocity = 100;
        srv.request.xyzAcceleration = 100;
        srv.request.rVelocity = 100;
        srv.request.rAcceleration = 100;
        client.call(srv);
    } while (0);

    // Set PTP jump parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPJumpParams>("/DobotServer/SetPTPJumpParams");
        dobot_msgs::SetPTPJumpParams srv;

        srv.request.jumpHeight = 20;
        srv.request.zLimit = 200;
        client.call(srv);
    } while (0);

    // Set PTP common parameters
    do {
        client = n.serviceClient<dobot_msgs::SetPTPCommonParams>("/DobotServer/SetPTPCommonParams");
        dobot_msgs::SetPTPCommonParams srv;

        srv.request.velocityRatio = 50;
        srv.request.accelerationRatio = 50;
        client.call(srv);
    } while (0);

    client = n.serviceClient<dobot_msgs::SetPTPCmd>("/DobotServer/SetPTPCmd");
    dobot_msgs::SetPTPCmd srv;

    while (ros::ok()) {
        // The first point
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 200;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }     
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);


        // The first point
        do {
            srv.request.ptpMode = 1;
            srv.request.x = 250;
            srv.request.y = 0;
            srv.request.z = 0;
            srv.request.r = 0;
            client.call(srv);
            if (srv.response.result == 0) {
                break;
            }
            ros::spinOnce();
            if (ros::ok() == false) {
                break;
            }
        } while (1);
  
        ros::spinOnce();
    }

    return 0;
}

