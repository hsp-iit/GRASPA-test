/*
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This software may be modified and distributed under the terms of the
* GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <cstdlib>
#include <string>

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

class ReachingTest : public RFModule, ReachingTest_IDL
{
    string port_prefix;
    string file_layout;

    pugi::xml_document parsed_file;

    // Poses to be reached
    vector<Vector> poses_layout;

    // Reached poses
    vector<Vector> reached_poses;

    // Port for thrift services
    RpcServer user_rpc;

    // Robot params
    string robot;
    string robot_arm;

    // Devices
    PolyDriver left_arm_client, right_arm_client;
    ICartesianControl *icart_right, *icart_left;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        port_prefix = "reaching-test";
        string log_ID = "[Configure]";

        file_layout = rf.check("file-layout", Value("layout_0.xml")).asString();

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
                    yError() << log_ID <<  "Could not open cartesian solver client for left arm";
                    return false;
                }
            }
            if ((robot_arm == "both") || (robot_arm == "right"))
            {
                if (!right_arm_client.open(optionRightArm))
                {
                    if (left_arm_client.isValid())
                    {
                        left_arm_client.close();
                    }
                    yError() << log_ID <<  "Could not open cartesian solver client for right arm";
                    return false;
                }
            }
        }

        // Open rpc port
        user_rpc.open("/" + port_prefix + "/cmd:rpc");

        //  attach callback
        attach(user_rpc);

        parse_xml(file_layout.c_str());

        return true;
    }

    /****************************************************************/
    bool updateModule()
    {
        return false;
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
    Bottle ask_new_pose()
    {

    }

    /****************************************************************/
    bool execute_new_pose()
    {

    }

    /****************************************************************/
    void parse_xml(const char* file)
    {
        yDebug() << "Parsing file: " << file;
        parsed_file.load_file(file);

        Matrix transform(4,4);
        vector<Matrix> poses;

        pugi::xml_node root = parsed_file.child("Scene");

        string namePanel;

        for (pugi::xml_node panel = root.first_child(); panel; panel = panel.next_sibling())
        {
            for (pugi::xml_node child = panel.first_child(); child; child = child.next_sibling())
            {
                for (pugi::xml_node attr = child.first_child(); attr; attr = attr.next_sibling())
                {
                    //yInfo() << attr.name();

                    int i = 0;

                    for (pugi::xml_node row = attr.first_child().first_child(); row; row = row.next_sibling())
                    {
                        //yInfo() << row.name();

                        int j = 0;

                        for (pugi::xml_attribute attr = row.first_attribute(); attr; attr = attr.next_attribute())
                        {
                            yInfo() << attr.name() << attr.value();
                            transform(i,j)=attr.as_double();
                            j++;
                        }

                        i++;
                    }

                    poses.push_back(transform);
                }
            }
        }

        for (size_t i = 0;  i < poses.size(); i++ )
        {
            yDebug()<< poses[i].toString();
        }
    }
};


int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    const std::string port_prefix = "reaching-test";

    std::unique_ptr<Network> yarp;
    yarp = std::move(std::unique_ptr<Network>(new Network()));

    if (!yarp->checkNetwork())
    {
            yError() << log_ID << "YARP seems unavailable!";
            return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("reaching-test");
    rf.setDefaultConfigFile("config.ini");
    rf.configure(argc, argv);

    ReachingTest module;

    return module.runModule(rf);
}
