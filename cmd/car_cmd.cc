
#include <iostream>
#include <string>
#include <vector>

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace gazebo;

int
main(int argc, char* argv[])
{
    if (argc != 3) {
        cout << "Usage: " << endl;
        cout << "\t" << argv[0] << " CMD NUM" << endl;
        return 1;
    }

    client::setup(argc, argv);
    transport::NodePtr node(new transport::Node());
    node->Init();

    auto cmd = string(argv[1]);
    cout << "cmd = " << cmd << endl;

    double xx = std::atof(argv[2]);
    cout << "xx0 = " << xx << endl;

    auto msg = msgs::ConvertAny(xx);
    cout << "xx1 = " << msg.double_value() << endl;

    string topic = "~/teapot0/bad_cmd";
    if (cmd == "turn") {
        topic = "~/teapot0/turn_cmd";
    }
    if (cmd == "vel") {
        topic = "~/teapot0/vel_cmd";
    }
    transport::PublisherPtr pub = node->Advertise<msgs::Any>(topic);

    pub->WaitForConnection();

    pub->Publish(msg);

    client::shutdown();
    return 0;
}
