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
#include "src/ArucoPoseEstimation_IDL.h"

using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace yarp::cv;

using namespace std;

/****************************************************************/
class ArucoPoseEstimation : public RFModule, ArucoPoseEstimation_IDL
{
    // Camera name
    string eye_name;
    // Hand name
    string hand_name;
    // Prefix for ports
    string port_prefix;
    // Aruco Dictionary
    string dictionary_string;
    // Switch between board and marker estimate poses
    bool use_board, found_dorso, found_side;
    // Number of markers in the board
    int n_markers_x, n_markers_y;
    // Marker id
    int marker_id_dorso, marker_id_side;
    // Dimensions and separation of markers in the board
    double marker_length, marker_separation;
    // Choose if to show image with the estimated pose
    bool send_image;
    // Check if marker side position is calibrated
    bool side_calibrated;
    // Matrix from marker on side to frame
    Matrix from_side_to_frame;
    // Dictionary of the marker
    cv::Ptr<cv::aruco::Dictionary> dictionary;
    // Aruco board
    cv::Ptr<cv::aruco::GridBoard> board;
    // A library with high level interface to iKinGazeCtrl
    GazeController *gaze;
    // Port for thrift services
    RpcServer user_rpc;
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
        hand_name = rf.check("hand_name", Value("right")).asString();
        send_image = rf.check("send_image", Value("True")).asBool();
        port_prefix = rf.check("port_prefix", Value("aruco-base-marker-estimation")).asString();
        dictionary_string = rf.check("aruco_dictionary", Value("4x4")).asString();
        n_markers_x = rf.check("n_markers_x", Value(5)).asInt();
        n_markers_y = rf.check("n_markers_y", Value(7)).asInt();
        marker_length = rf.check("marker_length", Value(0.05)).asDouble();
        marker_separation = rf.check("marker_separation", Value(0.038)).asDouble();
        use_board = rf.check("use_board", Value(true)).asBool();
        marker_id_dorso = rf.check("marker_id_dorso", Value(65)).asInt();
        marker_id_side = rf.check("marker_id_side", Value(65)).asInt();

        found_dorso = false;
        found_side = false;

        yInfo() << "=====================================";
        yInfo() << "eye_name " << eye_name;
        yInfo() << "hand_name " << hand_name;
        yInfo() << "n_markers_x " << n_markers_x;
        yInfo() << "n_markers_y " << n_markers_y;
        yInfo() << "marker_length " << marker_length;
        yInfo() << "marker_separation " << marker_separation;
        yInfo() << "dictionary_string " << dictionary_string;
        yInfo() << "use_board " << use_board;
        yInfo() << "marker_id_dorso " << marker_id_dorso;
        yInfo() << "marker_id_side " << marker_id_side;
        yInfo() << "=====================================";

        gaze = new::GazeController(port_prefix);

        //  attach callback
        attach(user_rpc);

        // Open rpc port
        user_rpc.open("/" + port_prefix + "/cmd:rpc");

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
        rf_ground_truth.setDefaultContext("aruco-pose-estimation");
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

        // We need to calibrate the matrix to transform side marker pose to hand pose
        yError() << log_ID_ << "Matrix from side marker to hand frame needs to be calibrated!";
        side_calibrated = false;
        from_side_to_frame.resize(4,4);
        from_side_to_frame(3,3) = 1;

        // Create boards (with same values as the printed one)
        board = cv::aruco::GridBoard::create(n_markers_x, n_markers_y, static_cast<float>(marker_length), static_cast<float>(marker_separation), dictionary);

        return true;
    }

    /*****************************************************************/
    bool attach(RpcServer &source)
    {
       return this->yarp().attachAsServer(source);
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

        found_dorso = false;
        found_side = false;

        // Perform marker detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        int valid = 0;

        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);
        for (size_t i = 0; i< ids.size(); i++)
        {
            if (ids[i] == marker_id_dorso || ids[i] == marker_id_side)
            {
        		valid = 1;
            	if (ids[i] == marker_id_dorso)
            	{
            	    found_dorso = true;
            	    break;
            	}
            	else if (side_calibrated == true)
            	{
            	    found_side = true;
            	    break;
            	}
            }
            else
        	valid = 0;
        }

        cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // Estimate pose of the board/markers
        cv::Vec3d rvec, tvec;
        vector<cv::Vec3d> rvecs, tvecs;

        if (use_board)
            valid = cv::aruco::estimatePoseBoard(corners, ids,  board, cam_intrinsic, cam_distortion, rvec, tvec);
        else
            cv::aruco::estimatePoseSingleMarkers(corners, marker_length, cam_intrinsic, cam_distortion, rvecs, tvecs);

        if (send_image)
        {
            // Draw the detected markers
            if(ids.size() > 0)
                cv::aruco::drawDetectedMarkers(image, corners, ids);
            // Draw the estimated pose of the board
            if (valid > 0 && use_board == true)
                cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec, tvec, 0.05);
            else if (valid > 0 && use_board == false)
            {
                for (size_t i = 0; i < ids.size(); i++)
                {
    	            if (side_calibrated == false)
    	            {
    		            if (ids[i] == marker_id_dorso)
    		            {
    		            	cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.05);
    		            	rvec = rvecs[i];
    		            	tvec = tvecs[i];
    		            }
                    }
                    else
                    {
                        if (found_dorso == true && ids[i] == marker_id_dorso)
                        {
                        	cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.05);
                        	rvec = rvecs[i];
                        	tvec = tvecs[i];
                        }
                        else if (found_side == true && ids[i] == marker_id_side)
                        {
                            cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.05);
                        	rvec = rvecs[i];
                        	tvec = tvecs[i];
                        }
                    }
                }
            }
            else
                yError() << "Not valid estimated board pose!";
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

        if (use_board == false)
        {
        	if (found_dorso == true)
        	{
                if (hand_name == "right")
                {
                    Vector direction(3,0.0);
        			direction = att_wrt_cam_yarp.subcol(0,0,3);
        			direction /= norm(direction);
        			pos_wrt_cam+= 0.025 * direction;

        			direction = att_wrt_cam_yarp.subcol(0,1,3);
        			direction /= norm(direction);
        			pos_wrt_cam += 0.0 * direction;

        			direction = att_wrt_cam_yarp.subcol(0,2,3);
        			direction /= norm(direction);
        			pos_wrt_cam -= 0.035 * direction;

        			Matrix R_around_x(3,3);
        			R_around_x.zero();
        			R_around_x(0,0) = 1.0;
        			R_around_x(1,1) = -1.0;
        			R_around_x(2,2) = -1.0;

        			att_wrt_cam_yarp = att_wrt_cam_yarp * R_around_x;
                }
                else
                {
                    // For left we just need to move the origin TODO check this
                    Vector direction(3,0.0);
        			direction = att_wrt_cam_yarp.subcol(0,0,3);
        			direction /= norm(direction);
        			pos_wrt_cam+= 0.025 * direction;

        			direction = att_wrt_cam_yarp.subcol(0,1,3);
        			direction /= norm(direction);
        			pos_wrt_cam += 0.0 * direction;

        			direction = att_wrt_cam_yarp.subcol(0,2,3);
        			direction /= norm(direction);
        			pos_wrt_cam -= 0.035 * direction;

                }

        	}
            else if (found_side == true && side_calibrated == true)
            {
                Matrix pos_omog_coord(4,4);
                pos_omog_coord.eye();
                pos_omog_coord.setSubcol(pos_wrt_cam, 0, 3);
                pos_omog_coord.setSubmatrix(att_wrt_cam_yarp, 0,0);

                pos_wrt_cam = (pos_omog_coord * from_side_to_frame).subcol(0,3,3);

                att_wrt_cam_yarp = att_wrt_cam_yarp * from_side_to_frame.submatrix(0,2,0,2);
            }
        }
        else
        {
        	Vector direction(3,0.0);
        	direction = att_wrt_cam_yarp.subcol(0,0,3);
        	direction /= norm(direction);
        	// TODO This needs to be fixed once we now the extact position of the markers w.r.t to marker frame in simox
        	pos_wrt_cam += (marker_length * n_markers_x + marker_separation * (n_markers_x-1)) * direction;
        }

        cv::Vec3d tvec_translated;
        tvec_translated[0] = pos_wrt_cam[0];
        tvec_translated[1] = pos_wrt_cam[1];
        tvec_translated[2] = pos_wrt_cam[2];

        cv::Vec3d rvec_rotated;
        cv::Mat att_wrt_cam_cv_rot(cv::Size(3,3), CV_64FC1);
        for (size_t i=0; i<3; i++)
        {
            for (size_t j=0; j<3; j++)
            {
                att_wrt_cam_cv_rot.at<double>(i, j) = att_wrt_cam_yarp(i, j);
            }
        }

        cv::Rodrigues(att_wrt_cam_cv_rot, rvec_rotated);

        if (send_image)
        {
    	    if (use_board == true && valid > 0)
    	    {
                cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec, tvec_translated, 0.1);
    	    }
    	    else if (valid > 0)
    	    {
        		for (size_t i=0; i< ids.size(); i++)
        		{
        			if (ids[i] == marker_id_dorso && found_dorso == true)
        				cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.1);
                    if (ids[i] == marker_id_side && found_side == true && side_calibrated == true)
        				cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.1);
        		}
                cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec_rotated, tvec_translated, 0.1);
    	    }

            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            port_image_out.write(false);
        }

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

        double t_send_pose = Time::now();
        // Send estimated pose via port
        Vector& pose_yarp = port_aruco_pose_out.prepare();
        pose_yarp.resize(7);

        pose_yarp.setSubvector(0,transform.getCol(3).subVector(0,2));
        pose_yarp.setSubvector(3, dcm2axis(transform.submatrix(0,2,0,2)));
        port_aruco_pose_out.write();

        return true;
    }

    /****************************************************************/
    bool reset_calibration()
    {
        from_side_to_frame.zero();
        side_calibrated = false;
        return true;
    }

    /****************************************************************/
    bool calibrate_markers()
    {
        string log_ID = "[calibrate_markers]";

        // Read from the camera
        ImageOf<PixelRgb>* image_in;
        image_in = port_image_in.read(true);
        if (image_in == nullptr)
    	{
    	    yError() << log_ID << "No image received!";
                return false;
    	}

        // Prepare output image
        ImageOf<PixelRgb>& image_out = port_image_out.prepare();

        // Copy input to output and wrap around a cv::Mat
        image_out = *image_in;
        cv::Mat image = yarp::cv::toCvMat(image_out);

        found_dorso = false;
        found_side = false;

        // Perform marker detection
        std::vector<int> ids;
        std::vector<std::vector<cv::Point2f> > corners, rejected;
        cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

        cv::aruco::detectMarkers(image, dictionary, corners, ids, parameters, rejected);
        for (size_t i = 0; i< ids.size(); i++)
        {
            if (ids[i] == marker_id_dorso)
        	{
        	    found_dorso = true;
        	}
        	else if (ids[i] == marker_id_side)
        	{
        	    found_side = true;
        	}
        }

        if (!(found_dorso == true && found_side == true))
    	{
    	    yError() << log_ID << "No markers detected!";
                return false;
    	}

        cv::aruco::refineDetectedMarkers(image, board, corners, ids, rejected);

        // Estimate pose
        cv::Vec3d rvec_side, tvec_side;
        cv::Vec3d rvec_dorso, tvec_dorso;
        vector<cv::Vec3d> rvecs, tvecs;
        cv::aruco::estimatePoseSingleMarkers(corners, marker_length, cam_intrinsic, cam_distortion, rvecs, tvecs);

        if (send_image)
        {
            // Draw the detected markers
            if(ids.size() > 0)
                cv::aruco::drawDetectedMarkers(image, corners, ids);
            // Draw the estimated pose of the board
            for (size_t i = 0; i < ids.size(); i++)
            {
                if (ids[i] == marker_id_dorso || ids[i] == marker_id_side)
                {
                	cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvecs[i], tvecs[i], 0.05);

                    if (ids[i] == marker_id_dorso)
                    {
                        rvec_dorso = rvecs[i];
                    	tvec_dorso = tvecs[i];
                    }

                    if (ids[i] == marker_id_side)
                    {
                        rvec_side = rvecs[i];
                    	tvec_side = tvecs[i];
                    }
                }
            }
        }

        // Get camera pose
        bool valid_camera_pose;
        Matrix camera_transform(4,4);
        valid_camera_pose = getCameraPose(camera_transform);
        if (!valid_camera_pose)
    	{
    	    yError() << log_ID << "No valid camera pose!";
                return false;
    	}

        // Compose dorso marker pose
        Vector pos_wrt_cam_dorso(3);
        pos_wrt_cam_dorso(0) = tvec_dorso(0);
        pos_wrt_cam_dorso(1) = tvec_dorso(1);
        pos_wrt_cam_dorso(2) = tvec_dorso(2);

        cv::Mat att_wrt_cam_cv_dorso;
        cv::Rodrigues(rvec_dorso, att_wrt_cam_cv_dorso);
        Matrix att_wrt_cam_yarp_dorso(3, 3);

        for (size_t i=0; i<3; i++)
            for (size_t j=0; j<3; j++)
                att_wrt_cam_yarp_dorso(i, j) = att_wrt_cam_cv_dorso.at<double>(i, j);

        // Compose side marker pose
        Vector pos_wrt_cam_side(3);
        pos_wrt_cam_side(0) = tvec_side(0);
        pos_wrt_cam_side(1) = tvec_side(1);
        pos_wrt_cam_side(2) = tvec_side(2);

        cv::Mat att_wrt_cam_cv_side;
        cv::Rodrigues(rvec_side, att_wrt_cam_cv_side);
        Matrix att_wrt_cam_yarp_side(3, 3);

        for (size_t i=0; i<3; i++)
            for (size_t j=0; j<3; j++)
                att_wrt_cam_yarp_side(i, j) = att_wrt_cam_cv_side.at<double>(i, j);

        Vector pos_hand_frame(3);
        pos_hand_frame = pos_wrt_cam_dorso;
        Matrix att_yarp_hand_frame(3,3);

        Vector direction(3,0.0);
    	direction = att_wrt_cam_yarp_dorso.subcol(0,0,3);
    	direction /= norm(direction);
    	pos_hand_frame += 0.025 * direction;

    	direction = att_wrt_cam_yarp_dorso.subcol(0,1,3);
    	direction /= norm(direction);
    	pos_hand_frame +=  0.0 * direction;

    	direction = att_wrt_cam_yarp_dorso.subcol(0,2,3);
    	direction /= norm(direction);
    	pos_hand_frame -=  0.035 * direction;

    	Matrix R_around_x(3,3);
    	R_around_x.zero();
    	R_around_x(0,0) = 1.0;
    	R_around_x(1,1) = -1.0;
    	R_around_x(2,2) = -1.0;
    	att_yarp_hand_frame = att_wrt_cam_yarp_dorso * R_around_x;

        // This is the hand frame obtained from dorso marker
        Matrix hand_frame(4,4);
        hand_frame(3,3) = 1.0;
        hand_frame.setSubmatrix(att_yarp_hand_frame, 0 ,0);
        hand_frame.setSubcol(pos_hand_frame, 0, 3);

        // Side marker pose
        Matrix homog_side_marker(4,4);
        homog_side_marker(3,3) = 1.0;
        homog_side_marker.setSubmatrix(att_wrt_cam_yarp_side, 0, 0);
        homog_side_marker.setSubcol(pos_wrt_cam_side, 0, 3);

        // This is the matrix to pass from the hand side marker to the hand frame
        from_side_to_frame = SE3inv(homog_side_marker) * hand_frame;

        // To check if it was correct
        // Compute quantities from dorso
        cv::Vec3d tvec_hand_from_dorso(3);
        tvec_hand_from_dorso(0) = pos_hand_frame(0);
        tvec_hand_from_dorso(1) = pos_hand_frame(1);
        tvec_hand_from_dorso(2) = pos_hand_frame(2);

        cv::Mat rvec_hand_from_dorso;
        cv::Mat att_cv_hand_frame_dorso(cv::Size(3,3), CV_64FC1);

        for (size_t i=0; i<3; i++)
        {
            for (size_t j=0; j<3; j++)
            {
                att_cv_hand_frame_dorso.at<double>(i, j) = hand_frame(i, j);
            }
        }
        cv::Rodrigues(att_cv_hand_frame_dorso, rvec_hand_from_dorso);

        // To check if it was correct
        // Compute quantities from side
        Vector pos_hand_frame_from_side(3, 0.0);
        Matrix homog_hand_from_side(4,4);
        homog_hand_from_side = homog_side_marker * from_side_to_frame;

        cv::Vec3d tvec_hand_from_side;
        tvec_hand_from_side(0) = homog_hand_from_side(0,3);
        tvec_hand_from_side(1) = homog_hand_from_side(1,3);
        tvec_hand_from_side(2) = homog_hand_from_side(2,3);

        cv::Mat rvec_hand_from_side;
        cv::Mat att_cv_hand_from_side(cv::Size(3,3), CV_64FC1);

        for (size_t i=0; i<3; i++)
        {
            for (size_t j=0; j<3; j++)
            {
                att_cv_hand_from_side.at<double>(i, j) = homog_hand_from_side(i, j);
            }
        }

        cv::Rodrigues(att_cv_hand_from_side, rvec_hand_from_side);

        side_calibrated = true;

        // Show everything
        if (send_image)
        {
            if (side_calibrated == true)
                cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec_side, tvec_side, 0.05);

            cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec_dorso, tvec_dorso, 0.05);

            cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec_hand_from_dorso, tvec_hand_from_dorso, 0.1);

            if (side_calibrated == true)
                cv::aruco::drawAxis(image, cam_intrinsic, cam_distortion, rvec_hand_from_side, tvec_hand_from_side, 0.1);

            cv::cvtColor(image, image, cv::COLOR_RGB2BGR);

            port_image_out.write(false);
        }

        yInfo() << log_ID << "Hand pose from side:";
        yInfo() << log_ID << (camera_transform * homog_hand_from_side).toString();

        yInfo() << log_ID << "Hand pose from dorso:";
        yInfo() << log_ID << (camera_transform * hand_frame).toString();

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

    std::unique_ptr<Network> yarp;
    yarp = std::move(std::unique_ptr<Network>(new Network()));

    if (!yarp->checkNetwork())
    {
        yError() << log_ID << "YARP seems unavailable!";
        return EXIT_FAILURE;
    }

    ResourceFinder rf;
    rf.setDefaultContext("aruco-pose-estimation");
    rf.setDefaultConfigFile("config_base.ini");
    rf.configure(argc, argv);

    ArucoPoseEstimation module;

    return module.runModule(rf);
}
