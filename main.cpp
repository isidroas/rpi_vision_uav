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
//#define MAV_CONNECT
//#define WRITE_IMAGES // very slow!
#define LOG_FILE

#define LOOP_PERIOD_MS  0  // Mínimo periodo del bucle

#define UPDATE_DEBUG_RATE  30 // Cada cuantas iteraciones se calculan e imprimen las estadísticas
#define WAIT_KEY_MILL      1 // tiempo de espera entre fotogramas cuando se abre la ventana, si vale 0, solo avanza cuando se presiona alguna tecla



int main(int argc, char** argv)
{
    cout << argc << endl;
    cout << argv << endl;

    #ifdef MAV_CONNECT	
    ComunicationClass commObj; 
    #endif
	 
    VisionClass  visionMarker; 
    
    double total_time = 0;
    int totalIterations = 0;
    int n_position_get = 0;
    auto x = std::chrono::steady_clock::now() + std::chrono::milliseconds(LOOP_PERIOD_MS);
    double tick_global_ant = (double)getTickCount();
    Eigen::Vector3d pos, euler_angles ;
    
    #ifdef LOG_FILE
    std::ofstream myfile;
    myfile.open("log.csv");
    myfile << "px" << "," << "py" << "," << "pz" << "," << "roll" << "," << "pitch" << "," << "yaw" << "," << "t" <<"\n";
    double seconds_init = (double)getTickCount()/getTickFrequency();
    #endif
  
    /*** Main Loop ***/
    while(true){

        double tick0 = (double)getTickCount();
        visionMarker.grab_and_retrieve_image();
        double tick1 = (double)getTickCount();
        // detect markers
        bool found_marker = visionMarker.detect_marker(pos, euler_angles);
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
            cout << "Marker detection" << execution_time_detect * 1000 << " ms " << endl;
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
            char key = (char)waitKey( WAIT_KEY_MILL );
        	if(key == 27) break;
        #endif
        #ifdef LOG_FILE
        double seconds = getTickCount()/ getTickFrequency() - seconds_init;
        myfile << pos[0] << "," << pos[1] << "," << pos[2] << "," << euler_angles[0] << "," << euler_angles[1] << "," << euler_angles[2] << "," << seconds << "\n";
        #endif


        std::this_thread::sleep_until(x);
        x = std::chrono::steady_clock::now() + std::chrono::milliseconds(LOOP_PERIOD_MS);
    }
    #ifdef LOG_FILE
    myfile.close();
    #endif
    std::cout << "Finished..." << std::endl;
    return EXIT_SUCCESS;
}
