/* mavsdk header */
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/mocap/mocap.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// vision header
#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

#include <Eigen/Dense>

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
//#include <iostream>

#define CALIBRATION_PARAMETERS "calibration_parameters.txt"
#define DICTIONARY 10   // 6x6 256
#define MARKER_LENGTH 0.173 
// Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1," "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}"
#define REFINEMENT_METHOD 1
#define SHOW_REJECTED  false
#define DEBUG
//#define MAV_CONNECT
#define OPEN_WINDOW

// The messages should be streamed at between 30Hz (33ms) (if containing covariances) and 50 Hz (20ms).
#define LOOP_PERIOD_MS  30 

using namespace std;
using namespace cv;

static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    fs["camera_matrix"] >> camMatrix;
    fs["distortion_coefficients"] >> distCoeffs;
    return true;
}


void wait_until_discover(Mavsdk& dc)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    dc.register_on_discover([&discover_promise](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discover_promise.set_value();
    });

    discover_future.wait();
}

void usage(std::string bin_name)
{
    std::cout << NORMAL_CONSOLE_TEXT << "Usage : " << bin_name << " <connection_url>" << std::endl
              << "Connection URL format should be :" << std::endl
              << " For TCP : tcp://[server_host][:server_port]" << std::endl
              << " For UDP : udp://[bind_host][:bind_port]" << std::endl
              << " For Serial : serial:///path/to/serial/dev[:baudrate]" << std::endl
              << "For example, to connect to the simulator use URL: udp://:14540" << std::endl;
}

// Checks if a matrix is a valid rotation matrix.
bool isRotationMatrix(Mat &R)
{
    Mat Rt;
    transpose(R, Rt);
    Mat shouldBeIdentity = Rt * R;
    Mat I = Mat::eye(3,3, shouldBeIdentity.type());
    
    return  norm(I, shouldBeIdentity) < 1e-6;
    
}

// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
Vec3d rotationMatrixToEulerAngles(Mat &R)
{

    assert(isRotationMatrix(R));
    
    float sy = sqrt(R.at<double>(0,0) * R.at<double>(0,0) +  R.at<double>(1,0) * R.at<double>(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R.at<double>(2,1) , R.at<double>(2,2));
        y = atan2(-R.at<double>(2,0), sy);
        z = atan2(R.at<double>(1,0), R.at<double>(0,0));
    }
    else
    {
        x = atan2(-R.at<double>(1,2), R.at<double>(1,1));
        y = atan2(-R.at<double>(2,0), sy);
        z = 0;
    }
    return Vec3f(x, y, z);
}

Eigen::Vector3d rotationMatrixToEulerAngles_eig(Eigen::Matrix3d &R)
{

    float sy = sqrt(R(0,0) * R(0,0) +  R(1,0) * R(1,0) );

    bool singular = sy < 1e-6; // If

    float x, y, z;
    if (!singular)
    {
        x = atan2(R(2,1) , R(2,2));
        y = atan2(-R(2,0), sy);
        z = atan2(R(1,0), R(0,0));
    }
    else
    {
        x = atan2(-R(1,2), R(1,1));
        y = atan2(-R(2,0), sy);
        z = 0;
    }
    return Eigen::Vector3d(x, y, z);
}


int main(int argc, char** argv)
{
    cout << argc << endl;
    cout << argv << endl;

    #ifdef MAV_CONNECT
    Mavsdk dc;
    std::string connection_url;
    ConnectionResult connection_result;

    if (argc == 2) {
        connection_url = argv[1];
        connection_result = dc.add_any_connection(connection_url);
    } else {
        usage(argv[0]);
        return 1;
    }

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    // Wait for the system to connect via heartbeat
    wait_until_discover(dc);

    // System got discovered.
    System& system = dc.system();
    auto action = std::make_shared<Action>(system);
    auto offboard = std::make_shared<Offboard>(system);
    auto telemetry = std::make_shared<Telemetry>(system);
    auto mocap = std::make_shared<Mocap>(system);
    #endif
	 
    /*** Vision setup ***/
    Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();

    //override cornerRefinementMethod read from config file
    detectorParams->cornerRefinementMethod = REFINEMENT_METHOD;
    std::cout << "Corner refinement method (0: None, 1: Subpixel, 2:contour, 3: AprilTag 2): " << detectorParams->cornerRefinementMethod << std::endl;

    int camId = 0;

    String video;

    Ptr<aruco::Dictionary> dictionary =
        aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(DICTIONARY));

    Mat camMatrix, distCoeffs;
        bool readOk = readCameraParameters(CALIBRATION_PARAMETERS, camMatrix, distCoeffs);
        if(!readOk) {
            cerr << "Invalid camera file" << endl;
            return 0;
        }

    VideoCapture inputVideo;
    int waitTime;
    if(!video.empty()) {
        inputVideo.open(video);
        waitTime = 0;
    } else {
        inputVideo.open(camId);
        waitTime = 10;
    }

    double total_time_detect = 0;
    double total_time = 0;
    int totalIterations = 0;
  
    /*** Main Loop ***/
    while(true){
      auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(LOOP_PERIOD_MS);
      double tick_global = (double)getTickCount();
      /* Vision task */
      inputVideo.grab();
      Mat image, imageCopy;
      inputVideo.retrieve(image);

      double tick = (double)getTickCount();

      vector< int > ids;
      vector< vector< Point2f > > corners, rejected;
      vector< Vec3d > rvecs, tvecs;
      //vector< Vector3d > rvecs, tvecs;

      // detect markers and estimate pose
      aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
      bool found_marker=ids.size() > 0;

      double execution_time_detect = ((double)getTickCount() - tick) / getTickFrequency();

      Mocap::VisionPositionEstimate  est_pos;

      if(found_marker)
         aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs, tvecs);

      #ifdef OPEN_WINDOW
         // draw results
         image.copyTo(imageCopy);
         if(found_marker){
            aruco::drawDetectedMarkers(imageCopy, corners, ids);
            aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[0], tvecs[0], MARKER_LENGTH * 0.5f);
         }

         if(SHOW_REJECTED && rejected.size() > 0)
             aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));
         imshow("out", imageCopy);
      #endif

      if (found_marker){
         // tvecs and rvecs are translation and rotation of the marker with respect camera axis
         // camera position coordinates are the same as autopilot
         
         // rvecs are rotation_vectors. Here it is transformed to rotation matrix
         cv::Mat  rot_mat;
         Rodrigues(rvecs[0],rot_mat);
         //if (!isRotationMatrix(rot_mat)){
         //    break;
         //}
         //Vec3d euler_angles = rotationMatrixToEulerAngles(rot_mat);

         Eigen::Matrix3d rot_mat_eig; 
         cv::cv2eigen(rot_mat,rot_mat_eig);


         // rotate marker axis in order to get z point down
         Eigen::Matrix3d  x_rotation;
         x_rotation << 1,  0,  0,
                       0, -1,  0,
                       0,  0, -1;
         Eigen::Matrix3d rot_mat_aux= rot_mat_eig*x_rotation; 

         // Get position and rotation of camera in marker axis
         //tvecs_trans=transpose(rot_mat)*tvecs[0];
         Eigen::Matrix3d rot_mat_inv;
         Eigen::Vector3d t_in(tvecs[0][0],tvecs[0][1],tvecs[0][2]);
         rot_mat_inv = rot_mat_eig.transpose();
         //vector<Vec3d> pos_inv;
         Eigen::Vector3d t_out = -rot_mat_inv*t_in;
         Eigen::Vector3d t_out2 = x_rotation*t_out;

         Eigen::Vector3d euler_angles_aux = rotationMatrixToEulerAngles_eig(rot_mat_aux);

         // This on get camera postion in marker coordinates (but is negated -> why?)
         est_pos.position_body.x_m = t_out2[0];
         est_pos.position_body.y_m = t_out2[1];
         est_pos.position_body.z_m = t_out2[2];

         est_pos.angle_body.roll_rad =  euler_angles_aux[0];
         est_pos.angle_body.pitch_rad = euler_angles_aux[1];
         est_pos.angle_body.yaw_rad =   euler_angles_aux[2];

         std::vector<float> covariance{NAN};
         est_pos.pose_covariance.covariance_matrix=covariance;

         #ifdef MAV_CONNECT
         Mocap::Result result= mocap->set_vision_position_estimate(est_pos);
         if(result!=Mocap::Result::Success){
             std::cerr << ERROR_CONSOLE_TEXT << "Set vision position failed: " << result << NORMAL_CONSOLE_TEXT << std::endl;
         }
         #endif
      }

      total_time_detect += execution_time_detect;
      double execution_time = ((double)getTickCount() - tick_global) / getTickFrequency();
      total_time += execution_time;
      totalIterations++;
      /* Print data */
      if(totalIterations % 30 == 0) {
         cout << "Detection Time = " << execution_time_detect * 1000 << " ms "
              << "(Mean = " << 1000 * total_time_detect / double(totalIterations) << " ms)" << endl;
         cout << "Execution Time = " << execution_time * 1000 << " ms " 
              << "(Mean = " << 1000 * total_time / double(totalIterations) << " ms)" << endl;
         #ifdef DEBUG
         if(found_marker)
            cout << est_pos << endl;
         #endif 
      }
      
      char key = (char)waitKey(waitTime);
      if(key == 27) break;

      //sleep_for(milliseconds(25)); //40Hz
      std::this_thread::sleep_until(x);
    }

    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
