/* mavsdk header */
#include <chrono>
#include <cmath>
#include <future>
#include <iostream>
#include <fstream>
#include <thread>
#include <unistd.h> 
#include <Eigen/Dense>
#include "marker_vision.h"
#include "mavlink_helper.h"

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace std;

#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue


#define DEBUG


#define UPDATE_DEBUG_RATE  30 // Cada cuantas iteraciones se calculan e imprimen las estadísticas

static bool readParameters(string filename, bool &mav_connect, bool &log_file, int &loop_period_ms) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    mav_connect = (string)fs["mav_connect"]=="true";
    log_file = (string)fs["log_file"]=="true";
    loop_period_ms = (int)fs["loop_period_ms"];
    cout << "Parámetros generales:" <<  endl;
    cout << "\tConexion mavlink:\t\t\t" <<  mav_connect << endl;
    cout << "\tLog de medidas:\t\t\t\t" <<  log_file << endl;
    cout << "\tPeriodo minimo de actualización:\t" <<  loop_period_ms << endl;
    cout << endl;
    return true;
}



int main()
{
    cout << "------------------------------------------" << endl;
    cout << "--------Vision position estimator---------" << endl;
    cout << "------------------------------------------" << endl;
    cout << endl;

    bool mav_connect, log_file;
    int loop_period_ms;
    readParameters("../vision_params.yml", mav_connect, log_file, loop_period_ms);

    ComunicationClass commObj;
    if (mav_connect)
        commObj.init();
	 
    VisionClass  visionMarker; 
    
    double total_time = 0;
    int totalIterations = 0;
    int n_position_get = 0;
    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(loop_period_ms);
    double tick_global_ant = (double)getTickCount();
    Eigen::Vector3d pos, euler_angles ;
    
    double seconds_init = (double)getTickCount()/getTickFrequency();

    std::ofstream myfile;
    if (log_file){
        myfile.open("log.csv");
        myfile << "px" << "," << "py" << "," << "pz" << "," << "roll" << "," << "pitch" << "," << "yaw" << "," << "t" <<"\n";
    }
  
    /*** Main Loop ***/
    while(true){

        double tick0 = (double)getTickCount();
        visionMarker.grab_and_retrieve_image();
        double tick1 = (double)getTickCount();
        // detect markers
        bool found_marker = visionMarker.detect_marker(pos, euler_angles);
        double tick2 = (double)getTickCount();

        if (found_marker){
            if (mav_connect)
	            commObj.send_msg(pos, euler_angles);
            n_position_get++;
        }

        #ifdef DEBUG
        // Update counters
        double tick_global_act = (double)getTickCount();
        double execution_time = (tick_global_act - tick_global_ant) / getTickFrequency();
        tick_global_ant = tick_global_act;
        double execution_time_detect = (tick2-tick1) / getTickFrequency();
        double execution_time_video_grab_and_ret = (tick1-tick0) / getTickFrequency();
        total_time += execution_time;
        totalIterations++;

        /* Print data every 30 frames = 1 seg approx*/
        if(totalIterations % UPDATE_DEBUG_RATE == 0) {
            cout << "Image grabbing and retrieving= " << execution_time_video_grab_and_ret * 1000 << " ms " << endl;
            cout << "Marker detection= " << execution_time_detect * 1000 << " ms " << endl;
            cout << "Execution time = " << execution_time * 1000 << " ms " 
                 << "(Mean = " << 1000 * total_time / float(UPDATE_DEBUG_RATE) << " ms)" << endl;
            cout << "Frames with position = " << n_position_get/float(UPDATE_DEBUG_RATE) * 100 << " \% " << endl;
	 
            if(found_marker){
               cout << "Estimated position:\t" <<       pos[0] << "\t" <<           pos[1] << "\t" <<           pos[2] << endl;
               cout << "Estimated orientation:\t" <<    euler_angles[0] << "\t" <<  euler_angles[1] << "\t" <<  euler_angles[2] << endl;
            }
            total_time=0;
            n_position_get=0;
	        cout << endl;
        }
        #endif 


        if (log_file){
            double seconds = getTickCount()/ getTickFrequency() - seconds_init;
            myfile << pos[0] << "," << pos[1] << "," << pos[2] << "," << euler_angles[0] << "," << euler_angles[1] << "," << euler_angles[2] << "," << seconds << "\n";
        }

        if (loop_period_ms!=0){
            std::this_thread::sleep_until(x);
            x = std::chrono::steady_clock::now() + std::chrono::milliseconds(loop_period_ms);
        }
    }

    if (log_file)
        myfile.close();
    std::cout << "Finished..." << std::endl;
    return EXIT_SUCCESS;
}
