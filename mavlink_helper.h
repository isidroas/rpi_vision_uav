void wait_until_discover(Mavsdk& dc)
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

auto mavlink_setup()
{
    Mavsdk dc;
    ConnectionResult connection_result;

    connection_result = dc.add_any_connection(CONNECTION_URL);

    if (connection_result != ConnectionResult::Success) {
        std::cout << ERROR_CONSOLE_TEXT << "Connection failed: " << connection_result
                  << NORMAL_CONSOLE_TEXT << std::endl;
        return 1;
    }

    bool connected = dc.is_connected(UUID);
    while(connected==false){
       connected = dc.is_connected(UUID);
       cout << "Waiting system for connection ..." << endl;
       sleep_for(milliseconds(500));
    }
    System& system = dc.system(UUID);

    auto mocap = std::make_shared<Mocap>(system);
    return mocap
}
