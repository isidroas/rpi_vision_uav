/* mavsdk header */
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <thread>
#include<unistd.h> 

#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mocap/mocap.h>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

// vision header
#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour
#define CONNECTION_URL  "serial:///dev/ttyUSB0:921600"
//#define UUID 3690507541151037490 // autopilot cube
#define UUID 3762846584429098293 // autopilot cuav

#include <Eigen/Dense>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define CALIBRATION_PARAMETERS "calibration_parameters.txt"
#define DICTIONARY 10   // 6x6 256
//#define MARKER_LENGTH 0.173 
#define MARKER_LENGTH 0.179 
// Corner refinement: CORNER_REFINE_NONE=0, CORNER_REFINE_SUBPIX=1," "CORNER_REFINE_CONTOUR=2, CORNER_REFINE_APRILTAG=3}"
#define REFINEMENT_METHOD 1
#define SHOW_REJECTED  false
#define DEBUG
#define MAV_CONNECT
//#define OPEN_WINDOW
//#define WRITE_IMAGES // very slow!
//#define DRAW_AXIS 

// The messages should be streamed at between 30Hz (33ms) (if containing covariances) and 50 Hz (20ms).
#define LOOP_PERIOD_MS  20 

// Print debug every 30 frames
#define UPDATE_DEBUG_RATE  30

using namespace std;
using namespace cv;

#include "marker_vision.h"
#include "mavlink_helper.h"


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


// Calculates rotation matrix to euler angles
// The result is the same as MATLAB except the order
// of the euler angles ( x and z are swapped ).
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
    commObj = ComunicationClass(); 
    #endif
	 
    visionMarker VisionClass(); 
    
    double total_time = 0;
    int totalIterations = 0;
    int n_position_get = 0;
    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(LOOP_PERIOD_MS);
    double tick_global_ant = (double)getTickCount();
    Eigen::Vector3d pos, euler_angles ;
  
    /*** Main Loop ***/
    while(true){

      double tick0 = (double)getTickCount();
      visionMarker.grab_and_retrieve();
      double tick1 = (double)getTickCount();
      // detect markers
      bool found_marker = visionMarker.detectMarker(pos, euler_angles)
      double tick2 = (double)getTickCount();

      if (found_marker){
        #ifdef MAV_CONNECT
	commObj.send_msg(pos, euler_angles);
        #endif
        n_position_get++;
      }
      #ifdef WRITE_IMAGES
      //char path [30];
      //sprintf(path,"./images/image%d.png", totalIterations);
      //imwrite(path,image);
      #endif

      #ifdef DEBUG
      // Update counters
      double tick_global_act = (double)getTickCount();
      double execution_time = (tick_global_act - tick_global_ant) / getTickFrequency();
      tick_global_ant = tick_global_act;
      double execution_time_detect = (tick2-tick1) / getTickFrequency();
      double execution_time_video_grab_and_ret = (tick1-tick0) / getTickFrequency();
      total_time += execution_time;
      totalIterations++;// TODO: eliminar esta variable ya que siempre valdrá 30

      /* Print data every 30 frames = 1 seg approx*/
      if(totalIterations % UPDATE_DEBUG_RATE == 0) {
         cout << "Image grabbing and retrieving= " << execution_time_video_grab_and_ret * 1000 << " ms " << endl;
         cout << "Execution time = " << execution_time * 1000 << " ms " 
              << "(Mean = " << 1000 * total_time / float(UPDATE_DEBUG_RATE) << " ms)" << endl;
         cout << "Frames with position = " << n_position_get/float(UPDATE_DEBUG_RATE) * 100 << " \% " << endl;
	 
         if(found_marker){
            cout << "Estimated position:\t" 	<< pos[0] << "\t" << pos[1] << "\t" << pos[2] << endl;
            cout << "Estimated orientation:\t" 	<< euler_angles[0] << "\t" << euler_angles[1] << "\t" << euler_angles[2] << endl;
         }
         total_time=0;
         totalIterations=0;
         n_position_get=0;
	 cout << endl;
      }
      #endif 
      #ifdef OPEN_WINDOW
      	 char key = (char)waitKey(10);
      	 if(key == 27) break;
      #endif

      std::this_thread::sleep_until(x);
      x = std::chrono::steady_clock::now() + std::chrono::milliseconds(LOOP_PERIOD_MS);
    }

    std::cout << "Finished..." << std::endl;

    return EXIT_SUCCESS;
}
