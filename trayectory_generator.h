#include <Eigen/Dense>


class ComunicationClass{
	public:
		void send_msg(Eigen::Vector3d pos, Eigen::Vector3d eul);
        void init();
	private:
		void wait_until_discover(Mavsdk& dc);
        shared_ptr<Mocap> mocap;
        Mocap::VisionPositionEstimate  est_pos;
        Mavsdk dc;
        ConnectionResult connection_result;
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
    connection_result = dc.add_any_connection(CONNECTION_URL);

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        exit(0);
    }

    bool connected = dc.is_connected(UUID);
    while(connected==false){
       connected = dc.is_connected(UUID);
       cout << "Waiting system for connection ..." << endl;
       sleep_for(milliseconds(500));
    }
    System& system = dc.system(UUID);

    mocap = std::make_shared<Mocap>(system);
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


