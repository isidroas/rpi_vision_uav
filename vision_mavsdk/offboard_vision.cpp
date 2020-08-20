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

#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>

#define CALIBRATION_PARAMETERS "calibration_parameters.txt"
#define DICTIONARY 10   // 6x6 256
#define MARKER_LENGTH 0.173 
#define REFINEMENT_METHOD 0
//"{refine   |       | Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1," "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}"

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


// Logs during Vision position
inline void mocap_log(const std::string msg)
{
    std::cout << "[Mocap] " << msg << std::endl;
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


int main(int argc, char** argv)
{
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

	
    /*  Send vision position estimate */
    mocap_log("Sending postition estimate");
    Mocap::VisionPositionEstimate  est_pos;
    est_pos.position_body.x_m =0;
    est_pos.position_body.y_m =0;
    est_pos.position_body.z_m =0;
    est_pos.angle_body.roll_rad =0;
    est_pos.angle_body.pitch_rad =0;
    est_pos.angle_body.yaw_rad =0;
    std::vector<float> covariance{NAN};
    est_pos.pose_covariance.covariance_matrix=covariance;

    /* Vision setup */
    bool showRejected = false;

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

    double totalTime = 0;
    int totalIterations = 0;
  
    /* Main Loop */
    for(unsigned int i=0; i<1000000; i++){
      /* Vision task */
      if (inputVideo.grab()){
         Mat image, imageCopy;
         inputVideo.retrieve(image);

         double tick = (double)getTickCount();

         vector< int > ids;
         vector< vector< Point2f > > corners, rejected;
         vector< Vec3d > rvecs, tvecs;

         // detect markers and estimate pose
         aruco::detectMarkers(image, dictionary, corners, ids, detectorParams, rejected);
         if(ids.size() > 0)
             aruco::estimatePoseSingleMarkers(corners, MARKER_LENGTH, camMatrix, distCoeffs, rvecs,
                                              tvecs);

         double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
         totalTime += currentTime;
         totalIterations++;
         if(totalIterations % 30 == 0) {
             cout << "Detection Time = " << currentTime * 1000 << " ms "
                  << "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
         }

         // draw results
         image.copyTo(imageCopy);
         if(ids.size() > 0) {
             aruco::drawDetectedMarkers(imageCopy, corners, ids);

                 for(unsigned int i = 0; i < ids.size(); i++)
                     aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvecs[i], tvecs[i],
                                     MARKER_LENGTH * 0.5f);
         }

         if(showRejected && rejected.size() > 0)
             aruco::drawDetectedMarkers(imageCopy, rejected, noArray(), Scalar(100, 0, 255));

         imshow("out", imageCopy);
         char key = (char)waitKey(waitTime);
         if(key == 27) break;
      }

      /* Publish task */
      Mocap::Result result= mocap->set_vision_position_estimate(est_pos);
      if(result!=Mocap::Result::Success){
          std::cerr << ERROR_CONSOLE_TEXT << "Set vision position failed: " << result << NORMAL_CONSOLE_TEXT << std::endl;
      }
      // The messages should be streamed at between 30Hz (if containing covariances) and 50 Hz.
      sleep_for(milliseconds(25)); //40Hz
    }

    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
