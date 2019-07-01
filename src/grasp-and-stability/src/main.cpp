/*
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This software may be modified and distributed under the terms of the
* GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <cstdlib>
#include <string>
#include <iomanip>
#include <sstream>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>

#include <yarp/dev/CartesianControl.h>
#include <yarp/dev/PolyDriver.h>

#include <pugixml.hpp>

#include "src/ReachingTest_IDL.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::dev;
using namespace yarp::math;

class GraspAndStability: public RFModule, GraspAndStability_IDL
{
    string port_prefix;
    string moving_arm;
    bool can_grasp;

    // Marker pose
    Matrix marker_pose_matrix;

    // Grasp pose to test
    Vector grasp_pose;

    // Port for thrift services
    RpcServer user_rpc;

    // Port to ask for grasp pose
    RpcClient grasp_pose_port;

    // Robot params
    string robot;
    string robot_arm;

    // Home positions
    Vector home_pos_left, home_orie_left;
    Vector home_pos_right, home_orie_right;

    // Devices
    PolyDriver left_arm_client, right_arm_client;
    ICartesianControl *icart_right, *icart_left;

    // BufferedPort for reading aruko poses
    BufferedPort<Vector> port_marker_pose_in;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        port_prefix = "grasp-and-stability";
        string log_ID = "[Configure]";

        // Read robot name
        if(!rf.check("robot"))
        {
            robot = (rf.check("sim")? "icubSim" : "icub");
        }
        else
        {
            robot = rf.find("robot").asString();
        }

        // Read robot arm
        robot_arm = rf.check("robot-arm", Value("right")).toString();

        // Open devices
        if((robot == "icubSim") || (robot == "icub"))
        {
            Property optionLeftArm, optionRightArm;

            optionLeftArm.put("device", "cartesiancontrollerclient");
            optionLeftArm.put("remote", "/" + robot + "/cartesianController/left_arm");
            optionLeftArm.put("local", "/" + port_prefix + "/cartesianClient/left_arm");

            optionRightArm.put("device", "cartesiancontrollerclient");
            optionRightArm.put("remote", "/" + robot + "/cartesianController/right_arm");
            optionRightArm.put("local", "/" + port_prefix + "/cartesianClient/right_arm");

            if ((robot_arm == "both") || (robot_arm == "left"))
            {
                if (!left_arm_client.open(optionLeftArm))
                {
                    if (left_arm_client.isValid())
                    {
                        left_arm_client.close();
                    }

                    yError() << log_ID <<  "Could not open cartesian solver client for left arm";
                    return false;
                }
                else
                {
                    left_arm_client.view(icart_left);

                    Vector dof;
                    icart_left->getDOF(dof);
                    Vector new_dof(10, 1);
                    new_dof(1) = 0.0;
                    icart_left->setDOF(new_dof, dof);
                    icart_left->setInTargetTol(0.001);
                    icart_left->setLimits(0, 0.0, 15.0);

                    icart_left->getPose(home_pos_left, home_orie_left);
                }
            }
            if ((robot_arm == "both") || (robot_arm == "right"))
            {
                if (!right_arm_client.open(optionRightArm))
                {
                    if (right_arm_client.isValid())
                    {
                        right_arm_client.close();
                    }
                    yError() << log_ID <<  "Could not open cartesian solver client for right arm";
                    return false;
                }
                else
                {
                    right_arm_client.view(icart_right);
                    Vector dof;
                    icart_right->getDOF(dof);
                    Vector new_dof(10, 1);
                    new_dof(1) = 0.0;
                    icart_right->setDOF(new_dof, dof);
                    icart_right->setInTargetTol(0.001);
                    icart_right->setLimits(0, 0.0, 15.0);

                    icart_right->getPose(home_pos_right, home_orie_right);
                }
            }
        }

        can_grasp = false;

        // Open rpc port
        user_rpc.open("/" + port_prefix + "/cmd:rpc");

        // Open port for aruko pose
        grasp_pose_port.open("/" + port_prefix + "/grasp_pose:rpc");

        // Open port for aruko pose
        port_marker_pose_in.open("/" + port_prefix + "/pose:in");

        //  attach callback
        attach(user_rpc);

        marker_pose_received = getMarkerPose();

        return marker_pose_received;
    }

    /****************************************************************/
    bool getMarkerPose()
    {
        Vector *marker_pose = port_marker_pose_in.read();

        if (marker_pose != NULL)
        {
            marker_pose_matrix.resize(4,4);
            marker_pose_matrix.zero();
            marker_pose_matrix.setSubmatrix(axis2dcm(marker_pose->subVector(3,6)), 0, 0);
            marker_pose_matrix.setSubcol(marker_pose->subVector(0,2), 0,3);
            marker_pose_matrix(3,3) = 1.0;

            yInfo() << log_ID << "Received marker pose (Vector): " << marker_pose->toString();
            yInfo() << log_ID << "Received marker pose (Matrix): " << marker_pose_matrix.toString();

            return true;
        }
        else
        {
            yError() << log_ID << " No marker pose received!";
            return false;
        }
    }

    /*****************************************************************/
    bool attach(RpcServer &source)
    {
       return this->yarp().attachAsServer(source);
    }

    /****************************************************************/
    bool updateModule()
    {
        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool interruptModule()
    {
        user_rpc.interrupt();
        return true;
    }

    /****************************************************************/
    bool close()
    {
        user_rpc.close();

        if (left_arm_client.isValid())
        {
            left_arm_client.close();
        }
        if (right_arm_client.isValid())
        {
            right_arm_client.close();
        }

        return true;
    }

    /****************************************************************/
    bool get_grasp(const string &arm)
    {
        string log_ID = "[get_grasp]";

        moving_arm = arm;

        Bottle cmd, reply;
        cmd.addString("grasp_pose");
        cmd.addString(arm);

        grasp_pose_port.write(cmd, reply);

        if (reply)
        {
            // TODO Ask Fabrizio to add rpc command to get the compute poses
            // from cardinal point grasps
            grasp.pose.resize(7,0.0);

            yInfo() << log_ID << "Received grasp pose " : grasp_pose.toString();

            can_grasp = true;
            return true;
        }
        else
        {
            yError() << log_ID << "Grasp pose not received!";
            can_grasp = false;
            return false;
        }
    }

    /****************************************************************/
    bool grasp()
    {
        string log_ID = "[grasp]";
        
        if (can_grasp)
        {
            bool reached_pose;

            // TODO Make grasp the object (reach and close hand)
            can_grasp = false;
            return reached_pose;
        }
        else
        {
            yError() << log_ID << "Grasp cannot be executed because no new pose has been computed";

            return false;
        }
    }
};

int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    const std::string port_prefix = "grasp-and-stability";

    std::unique_ptr<Network> yarp;
    yarp = std::move(std::unique_ptr<Network>(new Network()));

    if (!yarp->checkNetwork())
    {
            yError() << log_ID << "YARP seems unavailable!";
            return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("grasp-and-stability");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    GRaspAndStability module;

    return module.runModule(rf);
}
