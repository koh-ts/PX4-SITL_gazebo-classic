#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <chrono>
#include <thread>
#include <cmath>
#include <iostream>

int main(int argc, char** argv)
{
    std::string topic = "/gazebo/hitl_iris_world/wind"; // 必要に応じて変更
    double rate_hz = 20.0;
    double mean = 5.0;
    double amp  = 1.0;
    double freq = 0.05; // 0.05 Hz → 約20秒周期

    // 簡易引数: wind_publisher <topic> <mean> <amp> <freq> <rate>
    if (argc > 1) topic = argv[1];
    if (argc > 2) mean  = atof(argv[2]);
    if (argc > 3) amp   = atof(argv[3]);
    if (argc > 4) freq  = atof(argv[4]);
    if (argc > 5) rate_hz = atof(argv[5]);

    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();
    auto pub = node->Advertise<gazebo::msgs::Vector3d>(topic);
    pub->WaitForConnection();

    std::cout << "Publishing wind on " << topic
              << " (mean=" << mean << " amp=" << amp
              << " freq=" << freq << "Hz rate=" << rate_hz << "Hz)\n";

    auto t0 = std::chrono::steady_clock::now();
    while (true) {
        auto now = std::chrono::steady_clock::now();
        double t = std::chrono::duration<double>(now - t0).count();
        double speed = mean + amp * std::sin(2.0 * M_PI * freq * t);
        double dir   = 0.5 * std::sin(2.0 * M_PI * 0.05 * t) * M_PI; // ±90deg
        double vx = speed * std::cos(dir);
        double vy = speed * std::sin(dir);

        gazebo::msgs::Vector3d msg;
        msg.set_x(vx);
        msg.set_y(vy);
        msg.set_z(0.0);
        pub->Publish(msg);

        std::this_thread::sleep_for(std::chrono::milliseconds(int(1000.0 / rate_hz)));
    }
    gazebo::client::shutdown();
    return 0;
}