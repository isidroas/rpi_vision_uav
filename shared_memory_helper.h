using namespace std;
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <sys/mman.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <semaphore.h>
#include <string.h>
#include <sys/shm.h>
#include <Eigen/Dense>

#define SHARED_NAME   "/myshm"
#define SEMAPHORE_NAME "/mysem"
#define MEM_SIZE 1000



void report_and_exit(const char* msg) {
  perror(msg);
  exit(-1);
}
struct data_to_send{
  float x;
  float y;
  float z;
};

sem_t* semptr;
void* memptr;
int fd;


void shmem_init() {
// he añadido lo de crear con respecto al ejemplo
    fd = shm_open(SHARED_NAME, O_RDWR | O_CREAT, 0666);  /* empty to begin */
    if (fd < 0) report_and_exit("Can't get file descriptor...");

    ftruncate(fd, MEM_SIZE); /* get the bytes */

    /* get a pointer to memory */
    memptr = mmap(NULL,       /* let system pick where to put segment */
                          MEM_SIZE,   /* how many bytes */
                          PROT_READ | PROT_WRITE, /* access protections */
                          MAP_SHARED, /* mapping visible to other processes */
                          fd,         /* file descriptor */
                          0);         /* offset: start at 1st byte */
    if ((caddr_t) -1 == memptr) report_and_exit("Can't access segment...");

    sem_unlink(SEMAPHORE_NAME);
    /* create a semaphore for mutual exclusion */
    semptr = sem_open(SEMAPHORE_NAME, /* name */
                             O_CREAT| O_EXCL,       /* create the semaphore */
                             0666,   /* protection perms */
                             0);            /* initial value */
    //cout << "Se ha inicializado el semaforo" << endl;
    //cout << "semptr vale" << semptr <<  endl;
    if (semptr == (void*) -1) report_and_exit("sem_open");
    int semval;
    sem_getvalue(semptr,&semval);
    cout << "el semaforo vale " << semval  << endl;
    //cout << "Se ha inicializado la memoria compartida" << endl;
}



void shmem_cleanup(){
  /* cleanup */
  munmap(memptr, MEM_SIZE);
  close(fd);
  sem_close(semptr);
  sem_unlink(SEMAPHORE_NAME);
}

bool get_pos_from_tray_gen(Eigen::Vector3d &pos_setpoint){
    data_to_send *recv;

    //int semval;
    //sem_getvalue(semptr,&semval);
    //cout << "el semaforo vale " << semval  << endl;
    bool res=false;
    //cout << "El resultado de trywait es  " << res2 << endl;
    //cout << "eagain vale " << EAGAIN << endl;
    // doesn't lock
    /* use semaphore as a mutex (lock) by waiting for writer to increment it */
    if (!sem_trywait(semptr)) { /* wait until semaphore != 0 */
        recv=(data_to_send*)memptr;
        std::cout << recv->x << "\t" << recv->y << "\t" << recv->z << std::endl;
        pos_setpoint[0]=recv->x;
        pos_setpoint[1]=recv->y;
        pos_setpoint[2]=recv->z;
        sem_post(semptr);
        res = true;
    }
    return res;
}

void send_pos_ned_to_tray_gen(Eigen::Vector3d pos_ned){
    cout << "Posición NED del autopiloto: " << pos_ned << endl;
}
