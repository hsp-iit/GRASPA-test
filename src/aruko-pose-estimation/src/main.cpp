/*
* Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT)
*
* This software may be modified and distributed under the terms of the
* GPL-2+ license. See the accompanying LICENSE file for details.
*/

#include <cstdlib>
#include <string>

#include <opencv2/opencv.hpp>
#include <opencv2/aruco.hpp>

#include <yarp/os/all.h>
#include <yarp/sig/all.h>
#include <yarp/math/Math.h>
#include <yarp/dev/all.h>
#include <yarp/cv/Cv.h>

#include <GazeController.h>

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::cv;

using namespace std;

/****************************************************************/
class ArukoPoseEstimation : public RFModule
{
    // Camera name
    string eye_name;
    // Prefix for ports
    string port_prefix;
    // Aruko Dictionary
    string dictionary_string;
    // Number of markers in the board
    int n_markers_x, n_markers_y;
    // Dimensions and separation of markers in the board
    double marker_length, marker_separation;

    // Choose if to show image with the estimated pose
    bool send_image;

    // Dictionary of the marker
    cv::Ptr<cv::aruco::Dictionary> dictionary;

    // Aruco board
    cv::Ptr<cv::aruco::GridBoard> board;

    // A library with high level interface to iKinGazeCtrl
    GazeController *gaze;

    // Camera data
    cv::Mat cam_intrinsic;
    cv::Mat cam_distortion;

    // Ports for receiving and sending images
    BufferedPort<ImageOf<PixelRgb>> port_image_in;
    BufferedPort<ImageOf<PixelRgb>> port_image_out;

    // Streaming of the estimated pose
    BufferedPort<Vector> port_aruco_pose_out;

public:
    /****************************************************************/
    bool configure(ResourceFinder &rf)
    {
        // Basic sets
        string log_ID_ = "[Configure]";

        // Read data from config.ini
        eye_name = rf.check("eye_name", Value("right")).asString();
        send_image = rf.check("send_image", Value("True")).asBool();
        port_prefix = rf.check("port_prefix", Value("aruko-base-marker-estimation")).asString();
        dictionary_string = rf.check("aruko_dictionary", Value("4x4")).asString();
        n_markers_x = rf.check("n_markers_x", Value(5)).asInt();
        n_markers_y = rf.check("n_markers_y", Value(7)).asInt();
        marker_length = rf.check("marker_length", Value(0.05)).asDouble();
        marker_separation = rf.check("marker_separation", Value(0.038)).asDouble();

	yInfo() << "=====================================";
	yInfo() << "eye_name " << eye_name;
	yInfo() << "n_markers_x " << n_markers_x;
	yInfo() << "n_markers_y " << n_markers_y;
	yInfo() << "marker_length " << marker_length;
	yInfo() << "marker_separation " << marker_separation;
	yInfo() << "dictionary_string " << dictionary_string;
	yInfo() << "=====================================";

        gaze = new::GazeController(port_prefix);

        // Open camera  input port
        if(!port_image_in.open("/" + port_prefix + "/cam/" + eye_name + ":i"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            return false;
        }

        // Open camera  output port
        if(!port_image_out.open("/" + port_prefix + "/cam/" + eye_name + ":o"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open " + eye_name + " camera input port.";
            return false;
        }

        // Open aruco pose output port
        if(!port_aruco_pose_out.open("/" + port_prefix + "/marker-estimate/estimate:o"))
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open marker estimate output port.";
            return false;
        }

        // Find the path of the camera intrinsic parameters
        ResourceFinder rf_ground_truth;
        rf_ground_truth.setVerbose(true);
        rf_ground_truth.setDefaultContext("aruko-pose-estimation");
        std::string intrinsic_path = rf_ground_truth.findFile("opencv_icub_intrinsics_" + eye_name);

        // Load camera intrinsic parameters
        cv::FileStorage fs(intrinsic_path, cv::FileStorage::READ);
        if(!fs.isOpened())
        {
            std::string err = log_ID_ + "::CTOR::ERROR\n\tError: cannot open camera intrinsic parameters file.";
            return false;
        }

        fs["camera_matrix"] >> cam_intrinsic;
        fs["distortion_coefficients"] >> cam_distortion;

        // If available use camera intrinsics from the gaze controller
        double fx;
        double fy;
        double cx;
        double cy;
        bool valid_intrinsics = gaze->getCameraIntrinsics(eye_name, fx, fy, cx, cy);

        if (valid_intrinsics)
        {
            cam_intrinsic.at<double>(0, 0) = fx;
            cam_intrinsic.at<double>(0, 2) = cx;
            cam_intrinsic.at<double>(1, 1) = fy;
            cam_intrinsic.at<double>(1, 2) = cy;
        }

        // Configure a standard Aruco dictionary
        if (dictionary_string == "original")
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_ARUCO_ORIGINAL);
        else if (dictionary_string == "4x4")
            dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_50);

	// Create boards (with same values as the printed one)
        board = cv::aruco::GridBoard::create(n_markers_x, n_markers_y, marker_length, marker_separation, dictionary);

	return true;
    }

    /****************************************************************/
    bool close()
    {
        port_image_in.close();

        port_image_out.close();

        port_aruco_pose_out.close();

        delete gaze;

        return true;
    }

    /****************************************************************/
    double getPeriod()
    {
        return 0.1;
    }

    /****************************************************************/
    bool updateModule()
    {
        string log_ID = "[UpdateModule]";
        // Read from the camera
        ImageOf<PixelRgb>* image_in;

        image_in = port_image_in.read(true);


        if (image_in == nullptr)
            return true;

        // Prepare output image
        ImageOf<PixelRgb>& image_out = port_image_out.prepare();

        // Copy input to output and wrap around a cv::Mat
        image_out = *image_in;
        cv::Mat image = yarp::cv::toCvMat(image_out);

        // Perform marker detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
	cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
	
        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);

	cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // Estimate pose of the board
        cv::Vec3d rvec, tvec;
        int valid = cv::aruco::estimatePoseBoard(corners, ids,  board, cam_intrinsic, cam_distortion, rvec, tvec, true);


        if (send_image)
        {
            // Draw the detected markers
            if(ids.size() > 0)
                cv::aruco::drawDetectedMarkers(image, corners, ids);
            // Draw the estimated pose of the board
	    if (valid > 0 )
                 cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec, tvec, 0.1);
	    else
		 yError() << "Not valid estimated board pose!";

            // Send the image
            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            port_image_out.write();
        }

        // Get camera pose
        bool valid_camera_pose;
        Matrix camera_transform(4,4);
        valid_camera_pose = getCameraPose(camera_transform);
        if (!valid_camera_pose)
            return true;

        // Compose marker pose
        Vector pos_wrt_cam(3);
        pos_wrt_cam(0) = tvec(0);
        pos_wrt_cam(1) = tvec(1);
        pos_wrt_cam(2) = tvec(2);

        cv::Mat att_wrt_cam_cv;
        cv::Rodrigues(rvec, att_wrt_cam_cv);
        Matrix att_wrt_cam_yarp(3, 3);

        for (size_t i=0; i<3; i++)
            for (size_t j=0; j<3; j++)
                att_wrt_cam_yarp(i, j) = att_wrt_cam_cv.at<double>(i, j);

        // TODO translate origin of the board (to be tested)
        // Vector direction(3,0.0);
        // direction = att_wrt_cam_yarp.subcol(0,0,3) + att_wrt_cam_yarp.subcol(0,1,3);
        // direction /= norm(direction);
        // pos_wrt_cam += marker_length/2.0 * direction;
        // Matrix R_around_z(3,3);
        // R_around_z.zero();
        // R_around_z(0,1) = 1.0;
        // R_around_z(1,0) = -1.0;
        // R_around_z(2,2) = 1.0;
        // att_wrt_cam_yarp = R_around_z * att_wrt_cam_yarp;
        ///

        Matrix marker_transform(4,4);
        marker_transform.setSubcol(pos_wrt_cam, 0,3);
        marker_transform.setSubmatrix(att_wrt_cam_yarp, 0, 0);
	marker_transform(3,3) = 1.0;

        yInfo() << log_ID << "Transform from camera frame to marker frame ";
	yDebug() << marker_transform.toString();

        // Compose transform from root frame to marker frame
        Matrix transform = camera_transform * marker_transform;
	transform(3,3) = 1.0;

        yInfo() << log_ID << "Transform from root frame to marker frame ";
	yDebug() << transform.toString();

        // Send estimated pose via port
        Vector& pose_yarp = port_aruco_pose_out.prepare();
        pose_yarp.resize(7);

        pose_yarp.setSubvector(0,transform.getCol(3).subVector(0,2));
        pose_yarp.setSubvector(3, dcm2axis(transform.submatrix(0,2,0,2)));
        port_aruco_pose_out.write();

        return true;
    }

    /****************************************************************/
    bool getCameraPose(Matrix &camera_trans)
    {
        string log_ID = "[GetCameraPose] ";
        Vector eye_pos_left;
        Vector eye_att_left;

        Vector eye_pos_right;
        Vector eye_att_right;

        Vector eye_pos;
        Vector eye_att;

        camera_trans.eye();

        if (!gaze->getCameraPoses(eye_pos_left, eye_att_left, eye_pos_right, eye_att_right))
            return false;

        if (eye_name == "left")
        {
            eye_pos = eye_pos_left;
            eye_att = eye_att_left;
        }
        else
        {
            eye_pos = eye_pos_right;
            eye_att = eye_att_right;
        }


        camera_trans.setSubmatrix(axis2dcm(eye_att), 0, 0);
	camera_trans.setSubcol(eye_pos, 0,3);
	camera_trans(3,3) = 1.0;

        yDebug() << log_ID << "Transform from root frame to camera frame ";

	yDebug() << camera_trans.toString();

        return true;
    }

};

int main(int argc, char** argv)
{
    const std::string log_ID = "[Main]";
    yInfo() << log_ID << "Configuring and starting module...";

    //const std::string port_prefix = "aruko-pose-estimation";

    std::unique_ptr<Network> yarp;
    yarp = std::move(std::unique_ptr<Network>(new Network()));

    if (!yarp->checkNetwork())
    {
            yError() << log_ID << "YARP seems unavailable!";
            return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("aruko-pose-estimation");
    rf.setDefaultConfigFile("config_base.ini");
    rf.configure(argc, argv);

    ArukoPoseEstimation module;

    return module.runModule(rf);
}
