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

#define SHARED_NAME_SET   "/shm_set"
#define SEMAPHORE_NAME_SET "/sem_set"
#define MEM_SIZE_SET 1000

#define SHARED_NAME_EST   "/shm_est"
#define SEMAPHORE_NAME_EST "/sem_est"
#define MEM_SIZE_EST 1000


void report_and_exit(const char* msg) {
  perror(msg);
  exit(-1);
}
struct data_to_send{
  float x;
  float y;
  float z;
  float yaw;
  bool valid;
};

sem_t* semptr_set;
void* memptr_set;
int fd_setpoint;

sem_t* semptr_est;
void* memptr_est;
int fd_estimated;



/*********** Setpoints shared memory ***********/
void shmem_init_set() {
// he añadido lo de crear con respecto al ejemplo
    fd_setpoint = shm_open(SHARED_NAME_SET, O_RDWR | O_CREAT, 0666);  /* empty to begin */
    if (fd_setpoint < 0) report_and_exit("Can't get file descriptor...");

    ftruncate(fd_setpoint, MEM_SIZE_SET); /* get the bytes */

    /* get a pointer to memory */
    memptr_set = mmap(NULL,       /* let system pick where to put segment */
                          MEM_SIZE_SET,   /* how many bytes */
                          PROT_READ | PROT_WRITE, /* access protections */
                          MAP_SHARED, /* mapping visible to other processes */
                          fd_setpoint,         /* file descriptor */
                          0);         /* offset: start at 1st byte */
    if ((caddr_t) -1 == memptr_set) report_and_exit("Can't access segment...");

    sem_unlink(SEMAPHORE_NAME_SET);
    /* create a semaphore for mutual exclusion */
    semptr_set = sem_open(SEMAPHORE_NAME_SET, /* name */
                             O_CREAT| O_EXCL,       /* create the semaphore */
                             0666,   /* protection perms */
                             1);            /* initial value */
    //cout << "Se ha inicializado el semaforo" << endl;
    //cout << "semptr vale" << semptr <<  endl;
    if (semptr_set == (void*) -1) report_and_exit("sem_open");
    int semval;
    sem_getvalue(semptr_set,&semval);
    cout << "el semaforo vale " << semval  << endl;
    //cout << "Se ha inicializado la memoria compartida" << endl;

    data_to_send msg;
    msg.valid=false;
    memcpy((char*)memptr_set,(char*) &msg, sizeof(msg));
}

void shmem_cleanup(){
    /* cleanup */
    munmap(memptr_set, MEM_SIZE_SET);
    munmap(memptr_est, MEM_SIZE_EST);
    close(fd_setpoint);
    close(fd_estimated);
    sem_close(semptr_set);
    sem_close(semptr_est);
    sem_unlink(SEMAPHORE_NAME_SET);
    sem_unlink(SEMAPHORE_NAME_EST);
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
    if (!sem_trywait(semptr_set)) { /* wait until semaphore != 0 */
        recv=(data_to_send*)memptr_set;
        pos_setpoint[0]=recv->x;
        pos_setpoint[1]=recv->y;
        pos_setpoint[2]=recv->z;
        res = true and recv->valid;
        recv->valid=false;
        sem_post(semptr_set);
    }
    if (res)
        std::cout << "La posición setpoint del generador de trayectorias es " <<  recv->x << "\t" << recv->y << "\t" << recv->z << std::endl;

    return res;
}


/*********** Estimated shared memory ***********/
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

    sem_unlink(SEMAPHORE_NAME_EST);
    /* create a semaphore for mutual exclusion */
    semptr_est = sem_open(SEMAPHORE_NAME_EST, /* name */
                             O_CREAT| O_EXCL,       /* create the semaphore */
                             0666,   /* protection perms */
                             1);            /* initial value */
    //cout << "Se ha inicializado el semaforo" << endl;
    //cout << "semptr vale" << semptr <<  endl;
    if (semptr_est == (void*) -1) report_and_exit("sem_open");
    int semval;
    sem_getvalue(semptr_est,&semval);
    cout << "el semaforo vale " << semval  << endl;

    data_to_send msg;
    msg.valid=false;
    memcpy((char*)memptr_est,(char*) &msg, sizeof(msg));
    //sem_post(semptr);
    if (sem_post(semptr_est) < 0) report_and_exit("sem_post");
}

void send_pos_ned_to_tray_gen(Eigen::Vector3d pos_ned){
    data_to_send *env;
    if (!sem_trywait(semptr_est)) { /* wait until semaphore != 0 */
        env=(data_to_send*)memptr_est;
        env->x=pos_ned[0];
        env->y=pos_ned[1];
        env->z=pos_ned[2];
        env->valid=true;
        sem_post(semptr_est);
    }
    
}

void shmem_init(){
    shmem_init_set();
    shmem_init_est();
}
