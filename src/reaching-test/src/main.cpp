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
    string reached_poses_file;
    string port_marker_pose_out;
    int pose_count;

    // XML containing the poses for the reaching tests
    pugi::xml_document parsed_file;

    // Poses to be reached
    vector<Vector> poses_layout;

    // Marker pose
    Matrix marker_pose_matrix;

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

    // BufferedPort for reading aruko poses
    BufferedPort<Vector> port_marker_pose_in;

    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        port_prefix = "reaching-test";
        string log_ID = "[Configure]";

        file_layout = rf.check("file-layout", Value("layout_0.xml")).asString();
        reached_poses_file = rf.check("file-reached-poses", Value("test.xml")).asString();

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
                    if (right_arm_client.isValid())
                    {
                        right_arm_client.close();
                    }

                    yError() << log_ID <<  "Could not open cartesian solver client for left arm";
                    return false;
                }
                else
                {
                    left_arm_client.view(icart_left);
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
                }
            }
        }

        // Open rpc port
        user_rpc.open("/" + port_prefix + "/cmd:rpc");

        // Open port for aruko pose
        port_marker_pose_in.open("/" + port_prefix + "/pose:in");

        //  attach callback
        attach(user_rpc);

        // Read desired poses for the reaching test
        parseXml(file_layout.c_str());

        // Convert the pose using the marker pose
        bool marker_pose_received = convertPoses();
        pose_count = 0;

        return marker_pose_received;
    }

    /*****************************************************************/
    bool attach(RpcServer &source)
    {
       return this->yarp().attachAsServer(source);
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
    bool reset()
    {
        pose_count = 0;

        string log_ID = "[reset]";

        yInfo() << log_ID << "Reset pose no. ";
    }

    /****************************************************************/
    bool increase_pose()
    {
        string log_ID = "[increase_pose]";

        pose_count ++;

        yInfo() << log_ID << "Pose no. " << pose_count;
    }

    /****************************************************************/
    Vector ask_new_pose()
    {
        string log_ID = "[ask_new_pose]";

        yInfo() << log_ID << " Next pose to be executed: no. : "<< pose_count << ", value: " << poses_layout[pose_count].toString();

        return poses_layout[pose_count];
    }

    /****************************************************************/
    bool execute_new_pose(string &arm)
    {
        Vector reached_position(3,0.0);
        Vector reached_orientation(4,0.0);

        string log_ID = "[execute_new_pose]";

        if (arm == robot_arm || robot_arm == "both")
        {
            if (arm == "left")
            {
                yInfo() << log_ID << "Going to position: " << poses_layout[pose_count].subVector(0,2).toString() << "with orientation: " << poses_layout[pose_count].subVector(3,6).toString();
                icart_left->goToPoseSync(poses_layout[pose_count].subVector(0,2), poses_layout[pose_count].subVector(3,6));
                icart_left->waitMotionDone(0.4);
                icart_left->getPose(reached_position, reached_orientation);
            }
            else if (arm == "right")
            {
                yInfo() << log_ID << "Going to position: " << poses_layout[pose_count].subVector(0,2).toString() << "with orientation: " << poses_layout[pose_count].subVector(3,6).toString();
                icart_right->goToPoseSync(poses_layout[pose_count].subVector(0,2), poses_layout[pose_count].subVector(3,6));
                icart_right->waitMotionDone(0.4);
                icart_right->getPose(reached_position, reached_orientation);
            }

            yInfo() << log_ID << "Reached position: " << reached_position.subVector(0,2).toString() << "with orientation: " << reached_orientation.subVector(3,6).toString();

            Vector reached_pose(7,0.0);

            reached_pose.setSubvector(0, reached_position);
            reached_pose.setSubvector(3,reached_orientation);
            reached_poses.push_back(reached_pose);

            pose_count++;

            return true;
        }
        else
        {
            yError() << log_ID << "Not valid arm!";
            return false;
        }
    }

    /****************************************************************/
    bool save_reached_poses()
    {
        string log_ID = "save_reached_poses";

        //if (reached_poses.size() == poses_layout.size())
        if (1)
        {
            pugi::xml_document reched_poses_file;
            pugi::xml_node root = reched_poses_file.append_child("Scene");

            int j = 0;

            for (int i = 0; i< poses_layout.size(); i++)
            {
                string string_i = to_string(i%4);
                string string_j = to_string(j);
                pugi::xml_node object = root.append_child("ManipulationObject");
                object.append_attribute("name") = ("Reachable_frame"+string_j+string_i).c_str();

                yInfo() << "i" << i ;
                yInfo() << "modulo " << i%4;
                yInfo() << "j" << j ;

                if (i > 0)
                {
                    if (i%4 == 0)
                        j++;
                }
            }

            reched_poses_file.save_file(reached_poses_file.c_str());

            yInfo() << log_ID << "Poses saved in file: " << reached_poses_file;

            return true;
        }
        else
        {
            yError() << log_ID << "Not all the poses have been reached!";
            return false;
        }
    }

    /****************************************************************/
    void parseXml(const char* file)
    {
        string log_ID = "[ParseXml]";

        yDebug() << log_ID << "Parsing file: " << file;
        parsed_file.load_file(file);

        Matrix transform(4,4);
        vector<Matrix> poses;

        pugi::xml_node root = parsed_file.child("Scene");

        string namePanel;

        for (pugi::xml_node panel = root.first_child(); panel; panel = panel.next_sibling())
        {
            pugi::xml_attribute name_of_node = panel.first_attribute();

            if (string(name_of_node.value()) != "Table" && string(name_of_node.value()) != "Marker")
            {
                for (pugi::xml_node child = panel.first_child(); child; child = child.next_sibling())
                {
                    for (pugi::xml_node attr = child.first_child(); attr; attr = attr.next_sibling())
                    {
                        int i = 0;

                        for (pugi::xml_node row = attr.first_child().first_child(); row; row = row.next_sibling())
                        {
                            int j = 0;

                            for (pugi::xml_attribute attr = row.first_attribute(); attr; attr = attr.next_attribute())
                            {
                                transform(i,j)=attr.as_double();
                                j++;
                            }

                            i++;
                        }
                    }
                }

                yDebug()<< log_ID << transform.toString();

                poses.push_back(transform);

                Vector pose_7D(7, 0.0);
                pose_7D.setSubvector(0, transform.getCol(3).subVector(0,2));
                pose_7D.setSubvector(3, dcm2axis(transform.submatrix(0,2,0,2)));

                poses_layout.push_back(pose_7D);
            }
            else
                yInfo() << log_ID << "This is not a ManipulationObject node: skipped";
        }

        yDebug() << log_ID << "Total number of parsed poses " << poses.size();
    }

    /****************************************************************/
    bool convertPoses()
    {
        string log_ID = "[ConvertPoses]";

        Vector *marker_pose = port_marker_pose_in.read();

        if (marker_pose != NULL)
        {
            marker_pose_matrix.resize(4,4);
            marker_pose_matrix.setSubcol(marker_pose->subVector(0,2), 0,3);
            marker_pose_matrix.setSubmatrix(axis2dcm(marker_pose->subVector(3,6)), 0, 0);

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
