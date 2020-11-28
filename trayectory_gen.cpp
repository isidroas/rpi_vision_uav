#include <curses.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <semaphore.h>
#include <string.h>
#include <sys/shm.h>
#include <Eigen/Dense>

#define SHARED_NAME_SET   "/shm_set"
#define SEMAPHORE_NAME_SET "/sem_set"
#define MEM_SIZE_SET 1000

#define SHARED_NAME_EST   "/shm_est"
#define SEMAPHORE_NAME_EST "/sem_est"
#define MEM_SIZE_EST 1000

struct data_to_send{
  float x;
  float y;
  float z;
  bool valid;
};

void report_and_exit(const char* msg) {
  perror(msg);
  exit(-1);
}

int fd_setpoint;
void* memptr_set;
sem_t* semptr_set;

sem_t* semptr_est;
void* memptr_est;
int fd_estimated;

/********* Setpoints shared memory **********/
void shmem_init_set() {
    fd_setpoint = shm_open(SHARED_NAME_SET,      /* name from smem.h */
                      O_RDWR | O_CREAT, /* read/write, create if needed */
                      0666);     /* access permissions (0644) */
    if (fd_setpoint < 0) report_and_exit("Can't open shared mem segment...");

    ftruncate(fd_setpoint, MEM_SIZE_SET); /* get the bytes */ 
    memptr_set = mmap(NULL,       /* let system pick where to put segment */
                          MEM_SIZE_SET,   /* how many bytes */
                          PROT_READ | PROT_WRITE, /* access protections */
                          MAP_SHARED, /* mapping visible to other processes */
                          fd_setpoint,         /* file descriptor */
                          0);         /* offset: start at 1st byte */


    if ((caddr_t) -1  == memptr_set) report_and_exit("Can't get segment...");

    //fprintf(stderr, "shared mem address: %p [0..%d]\n", memptr, MEM_SIZE - 1);
    //fprintf(stderr, "backing file:       /dev/shm%s\n", SHARED_NAME );

    // Se supone que ya ha sido creado
    semptr_set = sem_open(SEMAPHORE_NAME_SET, /* name */
                             0,
                             0666,   /* protection perms */
                             0);            /* initial value */
    if (semptr_set == (void*) -1) report_and_exit("sem_open");

    //memcpy((char*)memptr_set,(char*) &msg, sizeof(msg));
    //sem_post(semptr);
    if (sem_post(semptr_set) < 0) report_and_exit("sem_post");

}

void shmem_init_est() {
// he añadido lo de crear con respecto al ejemplo
    fd_estimated = shm_open(SHARED_NAME_EST, O_RDWR | O_CREAT, 0666);  /* empty to begin */
    if (fd_estimated < 0) report_and_exit("Can't get file descriptor...");

    ftruncate(fd_estimated, MEM_SIZE_EST); /* get the bytes */

    /* get a pointer to memory */
    memptr_est = mmap(NULL,       /* let system pick where to put segment */
                          MEM_SIZE_EST,   /* how many bytes */
                          PROT_READ | PROT_WRITE, /* access protections */
                          MAP_SHARED, /* mapping visible to other processes */
                          fd_estimated,         /* file descriptor */
                          0);         /* offset: start at 1st byte */
    if ((caddr_t) -1 == memptr_est) report_and_exit("Can't access segment...");

    /* create a semaphore for mutual exclusion */
    semptr_est = sem_open(SEMAPHORE_NAME_EST, /* name */
                             0,       /* don't create the semaphore */
                             0666,   /* protection perms */
                             0);            /* initial value */
    //cout << "Se ha inicializado el semaforo" << endl;
    //cout << "semptr vale" << semptr <<  endl;
    if (semptr_est == (void*) -1) report_and_exit("sem_open");
}


void shmem_write(data_to_send msg){

    /* semaphore code to lock the shared mem */
    if (!sem_wait(semptr_set)) { /* wait until semaphore != 0 */
        //strcpy((char*)memptr,(char*) &msg); /* copy some ASCII bytes to the segment */
        memcpy((char*)memptr_set,(char*) &msg, sizeof(msg)); /* copy some ASCII bytes to the segment */

        /* increment the semaphore so that memreader can read */
        if (sem_post(semptr_set) < 0) report_and_exit("sem_post");
    }
}

bool shmem_read(data_to_send &msg){

    /* semaphore code to lock the shared mem */
    if (!sem_wait(semptr_est)) { /* wait until semaphore != 0 */
        //strcpy((char*)memptr,(char*) &msg); /* copy some ASCII bytes to the segment */
        memcpy((char*) &msg,(char*)memptr_est, sizeof(msg)); /* copy some ASCII bytes to the segment */

        /* increment the semaphore so that memreader can read */
        if (sem_post(semptr_est) < 0) report_and_exit("sem_post");
    }
    return msg.valid;
}




void shmem_cleanup (){
    /* clean up */
    munmap(memptr_set, MEM_SIZE_SET); /* unmap the storage */
    munmap(memptr_est, MEM_SIZE_EST); /* unmap the storage */
    close(fd_setpoint);
    close(fd_estimated);
    sem_close(semptr_set);
    sem_close(semptr_est);
    shm_unlink(SHARED_NAME_SET); /* unlink from the backing file */
    shm_unlink(SHARED_NAME_EST); /* unlink from the backing file */
}

int main(void) {
    initscr();
    noecho();
    cbreak();
    timeout(1000); // tiempo esperando a una tecla. -1 es para siempre
    //nodelay(stdscr, TRUE);
    
    int key;
      
    //shmem_init(); 

    key  = getch();

    printf("Generador de trayectorias\r\n");
    printf ("\t - Presiona 'q' para salir\r\n");
    printf ("\t - Presiona 's' para comenzar la misión\r\n");
    printf ("\t - Presiona 'w' para añadir un waypoint con espera\r\n");
    printf ("\t - Presiona 'i' para añadir un waypoint sin espera\r\n");

    shmem_init_est();
    shmem_init_set();
    while(true){
        data_to_send position_estimated;
        bool valid_pos_est = shmem_read(position_estimated);
        if (valid_pos_est)
            printf ("Posición NED recibida X: \t %f \r\n", position_estimated.x);
        key  = getch();
        if (key==113) break; 
        switch (key){
            case 119:
                printf("Se añade un waipoint\r\n");
                break;
            case 115:
                // s presionada
                printf("Se comienza una misión\r\n");
                data_to_send msg;
                msg.x=40;
                msg.y=41;
                msg.z=42;
                msg.z=42;
                msg.valid=true;
                shmem_write(msg);
                break;
            case 105:
                // i presionada
                printf("Se añade un waipoint intermedio\r\n");
                break;
            default:
                if (key!=-1)
                    printf ("%d %c\r\n", key, key);
        }

    }
    endwin(); // this restore terminal settings
    shmem_cleanup();
    return 0;
}
