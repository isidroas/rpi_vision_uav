#include "trayectory_gen.h"


int main(void) {
    WINDOW *win = initscr();
    noecho();
    //cbreak();
    raw();
    timeout(1000); // tiempo esperando a una tecla. -1 es para siempre
    //nodelay(stdscr, TRUE);
    scrollok(stdscr,TRUE);
    
    int key;
    bool mission_in_progress=false;
    int current_waypoint=0;
    int number_waypoints=0;
      
    //shmem_init(); 

    key  = getch();

    printw("Generador de trayectorias\n");
    printw("\t - Presiona 'q' para salir\n");
    printw("\t - Presiona 's' para comenzar la misión\n");
    printw("\t - Presiona 'w' para añadir un waypoint con espera\n");
    //printw("\t - Presiona 'i' para añadir un waypoint sin espera\n");
    printw("\t - Presiona 'e' para terminar la misión\n");
    printw("\t - Presiona 'n' avanzar al siguiente waypoint \n");
    refresh();

    int scr_x_ned, scr_y_ned;
    getyx(win, scr_y_ned, scr_x_ned);

    int  scr_x_set=scr_x_ned; 
    int  scr_y_set=scr_y_ned+1;

    int  scr_x_msg=scr_x_set; 
    int  scr_y_msg=scr_y_set+1;

    shmem_init_est();
    shmem_init_set();

    std::vector< Eigen::Vector3d > waypoints;

    while(true){
        data_to_send position_estimated;
        bool valid_pos_est = shmem_read(position_estimated);
        if (valid_pos_est)
            mvprintw(scr_y_ned,scr_x_ned,"Posición NED actual: \t %f \t %f \t %f \n", position_estimated.x, position_estimated.y, position_estimated.z);

        key  = getch();
        if (key==113) break; 
        switch (key){
            case 119:
                // 'w' pressed
                if (valid_pos_est){
                    Eigen::Vector3d newWaypoint(position_estimated.x, position_estimated.y, position_estimated.z); 
                    waypoints.push_back(newWaypoint); 
                    mvprintw(scr_y_msg, scr_x_msg, "Waypoint %d establecido en:\t %f \t %f \t %f \n", waypoints.size(), position_estimated.x, position_estimated.y, position_estimated.z);
                }
                else{
                    printw("No se tiene posición para añadir waypoint\n");
                }
                break;
            case 115:
                // s presionada
                printw("Se comienza una misión\n");
                mission_in_progress=true;
                current_waypoint=0;
                number_waypoints=waypoints.size();
                mvprintw(scr_y_msg, scr_x_msg, "Se avanza al waypoint %d\n", current_waypoint);
                break;
            case 101:
                // e presionada
                mvprintw(scr_y_msg, scr_x_msg, "Se termina la misión\n");
                mission_in_progress=false;
                break;
            case 110:
                // n presionada
                if (!mission_in_progress){
                    mvprintw(scr_y_msg, scr_x_msg, "No se ha iniciado la misión todavía\n");
                    break;
                }
                if (current_waypoint<number_waypoints){
                    mvprintw(scr_y_msg, scr_x_msg, "Se avanza al waypoint %d\n", current_waypoint);
                    current_waypoint++;
                }
                else{
                    mvprintw(scr_y_msg, scr_x_msg, "Se ha llegado al último Waypoint. Presione 'e' para terminar la mision\n");
                }
                break;
            //case 105:
            //    // i presionada
            //    printw("Se añade un waipoint intermedio\n");
            //    break;
            default:
                if (key!=-1)
                    mvprintw(scr_y_msg, scr_x_msg, "Se ha presionado una tecla inesperada: %d %c\n", key, key);
        }

        // Enviar waypoint
        if (mission_in_progress){
            data_to_send msg;
            msg.x=waypoints[current_waypoint][0];
            msg.y=waypoints[current_waypoint][1];
            msg.z=waypoints[current_waypoint][2];
            msg.valid=true;
            shmem_write(msg);
            mvprintw(scr_y_set,scr_x_set,"Posición de referencia: \t %f \t %f \t %f \n", msg.x, msg.y, msg.z);
        }
        else{
            mvprintw(scr_y_set,scr_x_set,"No se envía posición de referencia\n");
        }
        refresh();
    }
    endwin(); // this restore terminal settings
    shmem_cleanup();
    return 0;
}
