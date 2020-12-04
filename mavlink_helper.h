#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/mocap/mocap.h>
#include <Eigen/Dense>

using namespace mavsdk;
using std::chrono::milliseconds;
using std::this_thread::sleep_for;

//#define CONNECTION_URL  "serial:///dev/ttyUSB0:921600"
#define CONNECTION_URL  "udp://:14540"
//#define UUID 3690507541151037490 // autopilot cube
//#define UUID 3762846584429098293 // autopilot cuav
#define UUID 5283920058631409231 // simulation
#define ERROR_CONSOLE_TEXT "\033[31m" // Turn text on console red
#define NORMAL_CONSOLE_TEXT "\033[0m" // Restore normal console colour

static float pos_north=0;
static float pos_east=0;
static float pos_down=0;
static bool valid_ned=false;

class ComunicationClass{
	public:
        void send_msg(Eigen::Vector3d pos, Eigen::Vector3d eul);
        void send_pos_setpoint(Eigen::Vector3d );
        void init();
        bool get_pos_ned(Eigen::Vector3d &pos);
	private:
        void wait_until_discover(Mavsdk& dc);
        bool readCommParameters(string filename);
        shared_ptr<Mocap> mocap;
        shared_ptr<Telemetry> telemetry;
        shared_ptr<Offboard> offboard;
        Mocap::VisionPositionEstimate  est_pos;
        Mavsdk dc;
        ConnectionResult connection_result;
        uint64_t _uuid;
        string connection_url;
};


void ComunicationClass::wait_until_discover(Mavsdk& dc)
{
    std::cout << "Waiting to discover system..." << std::endl;
    std::promise<void> discover_promise;
    auto discover_future = discover_promise.get_future();

    dc.register_on_discover([&discover_promise](uint64_t uuid) {
        std::cout << "Discovered system with UUID: " << uuid << std::endl;
        discover_promise.set_value();
    });

    discover_future.wait();
}

void ComunicationClass::init()
{
    readCommParameters("../vision_params.yml");
    connection_result = dc.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(0);
    }

    bool connected = dc.is_connected(_uuid);
    while(connected==false){
       connected = dc.is_connected(_uuid);
       cout << "Waiting system for connection ..." << endl;
       sleep_for(milliseconds(500));
    }
    System& system = dc.system(_uuid);

    mocap = std::make_shared<Mocap>(system);
    telemetry = std::make_shared<Telemetry>(system);
    offboard = std::make_shared<Offboard>(system);

    // Set update rate
//    const Telemetry::Result set_rate_result = telemetry->set_rate_position(1.0);
//    if (set_rate_result != Telemetry::Result::Success) {
//        // handle rate-setting failure (in this case print error)
//        std::cout << "Setting rate failed:" << set_rate_result << std::endl;
//    }
    telemetry->subscribe_position_velocity_ned([](Telemetry::PositionVelocityNed posvel) {
        pos_north=posvel.position.north_m;
        pos_east=posvel.position.east_m;
        pos_down=posvel.position.down_m;
        valid_ned=true;
    });
}

void ComunicationClass::send_msg(Eigen::Vector3d pos, Eigen::Vector3d eul)
{
    est_pos.position_body.x_m = pos[0];
    est_pos.position_body.y_m = pos[1];
    est_pos.position_body.z_m = pos[2];
    est_pos.angle_body.roll_rad =  eul[0];
    est_pos.angle_body.pitch_rad = eul[1];
    est_pos.angle_body.yaw_rad =   eul[2];
    std::vector<float> covariance{NAN};
    est_pos.pose_covariance.covariance_matrix=covariance;
    Mocap::Result result= mocap->set_vision_position_estimate(est_pos);
    if(result!=Mocap::Result::Success){
        std::cerr << ERROR_CONSOLE_TEXT << "Set vision position failed: " << result << NORMAL_CONSOLE_TEXT << std::endl;
    }
}

void ComunicationClass::send_pos_setpoint(Eigen::Vector3d pos)
{
    Offboard::PositionNedYaw pos_setpoint{};
    pos_setpoint.north_m=pos[0];
    pos_setpoint.east_m=pos[1];
    pos_setpoint.down_m=pos[2];
    pos_setpoint.yaw_deg=0; // TODO: decide yaw. Se podrá Nan?
    Offboard::Result result = offboard->set_position_ned(pos_setpoint);
    if(result!=Offboard::Result::Success){
        std::cerr << ERROR_CONSOLE_TEXT << "Set offboard position setpoint failed: " << result << NORMAL_CONSOLE_TEXT << std::endl;
    }
}
bool ComunicationClass::get_pos_ned(Eigen::Vector3d &pos){
    pos[0]=pos_north;
    pos[1]=pos_east;
    pos[2]=pos_down;
    return valid_ned;
}

bool ComunicationClass::readCommParameters(string filename) {
    FileStorage fs(filename, FileStorage::READ);
    if(!fs.isOpened())
        return false;

    string uuid_aux = (string)fs["UUID"];
    _uuid = stoull(uuid_aux);
    connection_url = (string)fs["connection_url"];
    cout << "Parámetros de la comunicación:" <<  endl;
    cout << "\tUUID:\t\t" <<  _uuid << endl;
    cout << "\tconnection_url:\t" <<  connection_url << endl;
    cout << endl;
    return true;
}
