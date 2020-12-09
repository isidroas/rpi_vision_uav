#include "trayectory_gen.h"
#define WAIT_LOOP 1000 // Periodo de ejecución (ms)
#define WAIT_WP 1.0 // Tiempo de espera en el modo automatico (s)
#define DIST_THR 0.1 // umbral en el que se considera que se ha llegado al WP (m)

float get_current_time(){
    struct timespec tmv;
    clock_gettime(CLOCK_MONOTONIC, &tmv);
    float seconds = tmv.tv_sec + (double)tmv.tv_nsec/1e9;
    return seconds;
}

template<typename... Args>
void print_tm(float seconds, int scr_y,int scr_x,const char *msg,  Args... args){
    char msg2[200]="[%.1f] ";
    strcat(msg2,msg);
    mvprintw(scr_y,scr_x, msg2,seconds, args...);
}

int main(void) {
    WINDOW *win = initscr();
    noecho();
    //cbreak();
    raw();
    timeout(WAIT_LOOP); // tiempo esperando a una tecla. -1 es para siempre
    //nodelay(stdscr, TRUE);
    scrollok(stdscr,TRUE);
    
    int key;
    bool mission_in_progress=false;
    int current_waypoint=0;
    int number_waypoints=0;
    bool automatico=0;
    const float init_sec = get_current_time();
      
    //shmem_init(); 

    key  = getch();

    printw("Generador de trayectorias\n");
    printw("\t - Presiona 'q' para salir\n");
    printw("\t - Presiona 's' para comenzar la misión\n");
    printw("\t - Presiona 'w' para añadir un waypoint\n");
    //printw("\t - Presiona 'i' para añadir un waypoint sin espera\n");
    printw("\t - Presiona 'e' para terminar la misión\n");
    printw("\t - Presiona 'n' avanzar al siguiente waypoint \n");
    printw("\t - Presiona 'a' Para activar o desactivar el modo automatico \n\n");
    refresh();

    int scr_x_ned, scr_y_ned;
    getyx(win, scr_y_ned, scr_x_ned);

    int  scr_x_set=scr_x_ned; 
    int  scr_y_set=scr_y_ned+1;

    int  scr_x_mis=scr_x_set; 
    int  scr_y_mis=scr_y_set+1;

    int  scr_x_msg=scr_x_mis; 
    int  scr_y_msg=scr_y_mis+2;


    shmem_init_est();
    shmem_init_set();

    std::vector< Eigen::Vector3d > waypoints;

    while(true){

        // Esperar a que se presione una tecla
        key  = getch();

        data_to_send position_estimated;
        bool valid_pos_est = shmem_read(position_estimated);
        if (valid_pos_est)
            mvprintw(scr_y_ned,scr_x_ned,"Posición NED actual: \t %f \t %f \t %f \n", position_estimated.x, position_estimated.y, position_estimated.z);

        float seconds = get_current_time()-init_sec;

        if (key==113) break; 
        switch (key){
            case 119:
                // 'w' pressed
                if (valid_pos_est){
                    Eigen::Vector3d newWaypoint(position_estimated.x, position_estimated.y, position_estimated.z); 
                    waypoints.push_back(newWaypoint); 
                    //mvprintw(scr_y_msg, scr_x_msg, "Waypoint %d establecido en:\t %f \t %f \t %f \n", waypoints.size()-1, position_estimated.x, position_estimated.y, position_estimated.z);
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Waypoint %d establecido en:\t %f \t %f \t %f \n", waypoints.size()-1, position_estimated.x, position_estimated.y, position_estimated.z);
                }
                else{
                    print_tm(seconds, scr_y_msg, scr_x_msg,"No se tiene posición para añadir waypoint\n");
                }
                break;
            case 115:
                // s presionada
                printw("Se comienza una misión\n");
                mission_in_progress=true;
                current_waypoint=0;
                number_waypoints=waypoints.size();
                print_tm(seconds, scr_y_msg, scr_x_msg, "Se avanza al waypoint %d\n", current_waypoint);
                break;
            case 101:
                // e presionada
                print_tm(seconds, scr_y_msg, scr_x_msg, "Se termina la misión\n");
                mission_in_progress=false;
                break;
            case 110:
                // n presionada
                if (!mission_in_progress){
                    print_tm(seconds, scr_y_msg, scr_x_msg, "No se ha iniciado la misión todavía\n");
                    break;
                }
                if (current_waypoint<number_waypoints-1){
                    current_waypoint++;
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Se avanza al waypoint %d\n", current_waypoint);
                }
                else{
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Se ha llegado al último Waypoint. Presione 'e' para terminar la mision o 's' para volver a empezarla\n");
                }
                break;
            case 97:
                // a presionada
                if (automatico){
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Se desactiva el modo automatico\n");
                    automatico=false;
                }
                else{
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Se activa el modo automatico\n");
                    automatico=true;
                }
                break;
            default:
                if (key!=-1)
                    print_tm(seconds, scr_y_msg, scr_x_msg, "Se ha presionado una tecla inesperada: %d %c\n", key, key);
        }



        // Enviar waypoint
        if (mission_in_progress){
            data_to_send msg;
            msg.x=waypoints[current_waypoint][0];
            msg.y=waypoints[current_waypoint][1];
            msg.z=waypoints[current_waypoint][2];
            msg.valid=true;
            shmem_write(msg);

            // Distancia al WP
            //if (valid_pos_est){
            float x_diff= msg.x - position_estimated.x;
            float y_diff= msg.y - position_estimated.y;
            float z_diff= msg.z - position_estimated.z;
            float dist=sqrt(x_diff*x_diff + y_diff*y_diff + z_diff*z_diff); 
            if (automatico and dist< DIST_THR){
                // Next waypoint                
                if (current_waypoint<number_waypoints-1)
                    current_waypoint++;
                else 
                    current_waypoint=0;
            }
            

            mvprintw(scr_y_set,scr_x_set,"Posición de referencia: \t %f \t %f \t %f \t (waypoint %d) (distancia %f)\n", msg.x, msg.y, msg.z, current_waypoint,dist);
        }
        else{
            mvprintw(scr_y_set,scr_x_set,"No se envía posición de referencia\n");
        }

        if (mission_in_progress){
            if (automatico) 
                mvprintw(scr_y_mis,scr_x_mis,"Mision comenzada con %d waypoints (Modo automatico activado)\n", number_waypoints);
            else
                mvprintw(scr_y_mis,scr_x_mis,"Mision comenzada con %d waypoints\n", number_waypoints);
        }
        else {
            mvprintw(scr_y_mis,scr_x_mis,"No se ha iniciado ninguna mision\n");
        }

        refresh();
    }
    endwin(); // this restore terminal settings
    shmem_cleanup();
    return 0;
}
