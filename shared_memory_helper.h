using namespace std;

void get_pos_from_tray_gen(Eigen::Vector3d &pos_setpoint){
    pos_setpoint[0]=0;
    pos_setpoint[1]=0;
    pos_setpoint[2]=-2;
}
void send_pos_ned_to_tray_gen(Eigen::Vector3d pos_ned){
    cout << "Posición NED del autopiloto: " << pos_ned << endl;
}
