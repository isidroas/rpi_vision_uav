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
#include "shared_memory_helper.h"

using std::chrono::milliseconds;
using std::chrono::seconds;
using std::this_thread::sleep_for;

using namespace std;

#define TELEMETRY_CONSOLE_TEXT "\033[34m" // Turn text on console blue


#define DEBUG


#define UPDATE_DEBUG_RATE  30 // Cada cuantas iteraciones se calculan e imprimen las estadísticas

static bool readParameters(string filename, bool &mav_connect, bool &log_file, int &loop_period_ms, int &wait_key_mill, bool &open_window, bool &vision_activated, bool &tray_gen) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;
    mav_connect = (string)fs["mav_connect"]=="true";
    log_file = (string)fs["log_file"]=="true";
    vision_activated = (string)fs["vision_activated"]=="true";
    open_window = (string)fs["open_window"]=="true";
    tray_gen = (string)fs["tray_gen"]=="true";
    loop_period_ms = (int)fs["loop_period_ms"];
    wait_key_mill = (int)fs["wait_key_mill"];
    cout << "Parámetros generales:" <<  endl;
    cout << "\tConexion mavlink:\t\t\t" <<  mav_connect << endl;
    cout << "\tLog de medidas:\t\t\t\t" <<  log_file << endl;
    cout << "\tPeriodo minimo de actualización:\t" <<  loop_period_ms << endl;
    cout << "\tEspera a la presión de una tecla:\t" <<  wait_key_mill << endl;
    cout << endl;
    return true;
}



int main()
{
    cout << "------------------------------------------" << endl;
    cout << "--------Vision position estimator---------" << endl;
    cout << "------------------------------------------" << endl;
    cout << endl;

    /* Startup python script for logging */
    int res=system("python3 ../python_scripts/startup.py");
    if (res!=0){
        cout << "El script de inicio ha fallado con código " << res << endl;
        exit(1);
    }

    bool mav_connect, log_file, open_window, vision_activated, tray_gen;
    int loop_period_ms, wait_key_mill;
    readParameters("../vision_params.yml", mav_connect, log_file, loop_period_ms, wait_key_mill, open_window, vision_activated, tray_gen);

    ComunicationClass commObj;
    if (mav_connect)
        commObj.init();
	 
    
    double total_time = 0;
    int totalIterations = 0;
    auto wake_up_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(loop_period_ms);
    double tick_global_ant = (double)getTickCount();
    Eigen::Vector3d pos, euler_angles ;
    
    double seconds_init = (double)getTickCount()/getTickFrequency();

    VisionClass  visionMarker; 
    if (vision_activated)
        visionMarker.init(); 


    std::ofstream myfile;
    if (log_file){
        myfile.open("../results/latest/log.csv");
        myfile << "px" << "," << "py" << "," << "pz" << "," << "roll" << "," 
                       << "pitch" << "," << "yaw" << "," << "t" <<"\n";
    }

    // Inicializa la memoria compartida 
    if (tray_gen)
        shmem_init();
  
    /*** Main Loop ***/
    while(true){

        if (vision_activated){
            // Grab image and exists if there is no one
            if (!visionMarker.grab_and_retrieve_image()) break;
            // detect markers
            bool found_marker = visionMarker.detect_marker(pos, euler_angles);

            if (found_marker and mav_connect)
	                commObj.send_msg(pos, euler_angles);
            
        }
        
        //TODO: hacer esto con menor frecuencia
        // Get position setpoint from Trayectory Generator
        Eigen::Vector3d pos_setpoint;
        if (tray_gen)
            get_pos_from_tray_gen(pos_setpoint); 

        if (mav_connect and tray_gen){
            // Send setpoint to autopilot
            commObj.send_pos_setpoint(pos_setpoint);

            // Get position from autopilot. Not blocking function
            Eigen::Vector3d pos_ned;
            bool valid_ned = commObj.get_pos_ned(pos_ned);

            // Send ned position to Trayectory Generator
            if (valid_ned)
                send_pos_ned_to_tray_gen(pos_ned);
                //cout << "Posición NED: " << pos_ned;
        }


        #ifdef DEBUG
        // Update counters
        double tick_global_act = (double)getTickCount();
        double execution_time = (tick_global_act - tick_global_ant) / getTickFrequency();
        tick_global_ant = tick_global_act;
        total_time += execution_time;
        totalIterations++;

        /* Print data every 30 frames = 1 seg approx*/
        if(totalIterations % UPDATE_DEBUG_RATE == 0) {
            cout << "Execution time = " << execution_time * 1000 << " ms " 
                 << "(Mean = " << 1000 * total_time / float(UPDATE_DEBUG_RATE) << " ms)" << endl;
            if (vision_activated)
                visionMarker.print_statistics(pos, euler_angles);
            total_time=0;
	        cout << endl;
        }
        #endif 


        if (log_file){
            double seconds = getTickCount()/ getTickFrequency() - seconds_init;
            myfile << pos[0] << "," << pos[1] << "," << pos[2] << "," << euler_angles[0] << "," << 
                              euler_angles[1] << "," << euler_angles[2] << "," << seconds << "\n";
        }

        if (loop_period_ms!=0){
            std::this_thread::sleep_until(wake_up_time);
            wake_up_time = std::chrono::steady_clock::now() + std::chrono::milliseconds(loop_period_ms);
        }

        if (open_window){
            int key =waitKey( wait_key_mill );
            if(key == 27) break; // exits if esc is pressed in window
        }
    }

    if (log_file)
        myfile.close();

    res=system("python3 ../python_scripts/shutdown.py");
    if (res!=0){
        cout << "El script de apagado ha fallado con código " << res << endl;
        exit(1);
    }
    if (tray_gen)
        shmem_cleanup();
    std::cout << "Finished..." << std::endl;
    return EXIT_SUCCESS;
}
