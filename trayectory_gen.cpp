#include <curses.h>
#include <unistd.h>



int main(void) {
  initscr();
  noecho();
  cbreak();
  timeout(1000); // tiempo esperando a una tecla. -1 es para siempre
  //nodelay(stdscr, TRUE);
  
  int key;

  key  = getch();

  printf("Generador de trayectorias\r\n");
  printf ("\t - Presiona 'q' para salir\r\n");
  printf ("\t - Presiona 's' para comenzar la misión\r\n");
  printf ("\t - Presiona 'w' para añadir un waypoint con espera\r\n");
  printf ("\t - Presiona 'i' para añadir un waypoint sin espera\r\n");

  while(true){
    key  = getch();
    if (key==113) break; 
    switch (key){
        case 119:
            printf("Se añade un waipoint\r\n");
            break;
        case 115:
            // s presionada
            printf("Se comienza una misión\r\n");
            break;
        case 105:
            // i presionada
            printf("Se añade un waipoint intermedio\r\n");
            break;
        default:
            if (key!=-1)
                printf ("%d %c\r\n", key, key);
    }

    // sleep
  }
  endwin(); // this restore terminal settings
  return 0;
}
