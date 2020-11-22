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

#define SHARED_NAME   "/myshm"
#define SEMAPHORE_NAME "/mysem"
#define MEM_SIZE 1000


struct data_to_send{
  float x;
  float y;
  float z;
};
void report_and_exit(const char* msg) {
  perror(msg);
  exit(-1);
}

int fd;
void* memptr;
sem_t* semptr;

void shmem_init(data_to_send msg) {
    fd = shm_open(SHARED_NAME,      /* name from smem.h */
                      O_RDWR | O_CREAT, /* read/write, create if needed */
                      0666);     /* access permissions (0644) */
    if (fd < 0) report_and_exit("Can't open shared mem segment...");

    ftruncate(fd, MEM_SIZE); /* get the bytes */
    
    memptr = mmap(NULL,       /* let system pick where to put segment */
                          MEM_SIZE,   /* how many bytes */
                          PROT_READ | PROT_WRITE, /* access protections */
                          MAP_SHARED, /* mapping visible to other processes */
                          fd,         /* file descriptor */
                          0);         /* offset: start at 1st byte */


    if ((caddr_t) -1  == memptr) report_and_exit("Can't get segment...");

    //fprintf(stderr, "shared mem address: %p [0..%d]\n", memptr, MEM_SIZE - 1);
    //fprintf(stderr, "backing file:       /dev/shm%s\n", SHARED_NAME );

    // Se supone que ya ha sido creado
    semptr = sem_open(SEMAPHORE_NAME, /* name */
                             0,
                             0666,   /* protection perms */
                             0);            /* initial value */
    if (semptr == (void*) -1) report_and_exit("sem_open");

    memcpy((char*)memptr,(char*) &msg, sizeof(msg)); /* copy some ASCII bytes to the segment */
    //sem_post(semptr);
    if (sem_post(semptr) < 0) report_and_exit("sem_post");

}

void shmem_write(data_to_send msg){
    //data_to_send msg;
    //msg.x=0;
    //msg.y=2;
    //msg.z=1;

    /* semaphore code to lock the shared mem */
    if (!sem_wait(semptr)) { /* wait until semaphore != 0 */
        //strcpy((char*)memptr,(char*) &msg); /* copy some ASCII bytes to the segment */
        memcpy((char*)memptr,(char*) &msg, sizeof(msg)); /* copy some ASCII bytes to the segment */

        /* increment the semaphore so that memreader can read */
        if (sem_post(semptr) < 0) report_and_exit("sem_post");
    }
}

void shmem_cleanup (){
    /* clean up */
    munmap(memptr, MEM_SIZE); /* unmap the storage */
    close(fd);
    sem_close(semptr);
    shm_unlink(SHARED_NAME); /* unlink from the backing file */
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

    bool shmem_initialized = false;

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
                data_to_send msg;
                msg.x=40;
                msg.y=41;
                msg.z=42;
                if (shmem_initialized)
                    shmem_write(msg);
                else
                    shmem_init(msg);
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
