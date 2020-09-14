
#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using std::string;
using std::vector;
using namespace gazebo;
using physics::ModelPtr;
using physics::JointControllerPtr;
using ignition::math::Pose3d;
using common::Time;

class CarControlPlugin : public ModelPlugin
{
public:
    physics::ModelPtr model;
    vector<string> drive_joints;
    vector<string> steer_joints;

    transport::NodePtr node;

    transport::SubscriberPtr vel_sub;
    transport::SubscriberPtr turn_sub;
    transport::SubscriberPtr stat_sub;

    transport::PublisherPtr  pose_pub;

    CarControlPlugin() {}

    void
    SetSpeed(double speed)
    {
        auto jc = model->GetJointController();
        for (auto name : this->drive_joints) {
            jc->SetVelocityTarget(name, speed);
        }
    }

    void
    SetTurn(double turn)
    {
        auto jc = model->GetJointController();
        for (auto name : this->steer_joints) {
            jc->SetPositionTarget(name, turn);
        }
    }

    void
    SetPosPID(string name)
    {
        auto pid = common::PID(0.10, 0.01, 0.01);
        auto jc = model->GetJointController();
        jc->SetPositionPID(name, pid);
    }

    void
    SetVelPID(string name)
    {
        auto pid = common::PID(0.15, 0, 0);
        auto jc = model->GetJointController();
        jc->SetVelocityPID(name, pid);

    }

    virtual void
    Load(physics::ModelPtr model, sdf::ElementPtr sdf)
    {
        this->model = model;

        if (model->GetJointCount() == 0) {
            std::cerr << "bad model loaded" << std::endl;
        }

        auto model_name = this->model->GetName();
        auto world_name = this->model->GetWorld()->Name();

        std::cerr << "hello from ControlPlugin" << std::endl;
        std::cerr << "world: " << world_name << std::endl;
        std::cerr << "model: " << model_name << std::endl;

        for (auto joint : model->GetJoints()) {
            auto name = joint->GetName();
            auto sname = joint->GetScopedName();

            if (name == std::string("teapot::wheel_rl_drive") ||
                name == std::string("teapot::wheel_rr_drive")) {
                this->drive_joints.push_back(sname);
                this->SetVelPID(sname);
            }

            if (name == std::string("teapot::wheel_fl_steer") ||
                name == std::string("teapot::wheel_fr_steer"))
            {
                this->steer_joints.push_back(sname);
                this->SetPosPID(sname);
            }

            std::cerr << "joint: " << joint->GetName() << std::endl;
        }

        this->SetSpeed(0.0);
        this->SetTurn(0.0);

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(world_name);

        string vel_cmd_topic = "~/" + model_name + "/vel_cmd";
        this->vel_sub = this->node->Subscribe(
            vel_cmd_topic, &CarControlPlugin::OnVelCmd, this);
        std::cerr << "Subscribed vel_cmd" << std::endl;

        string turn_cmd_topic = "~/" + model_name + "/turn_cmd";
        this->turn_sub = this->node->Subscribe(
            turn_cmd_topic, &CarControlPlugin::OnTurnCmd, this);
        std::cerr << "Subscribed turn_cmd" << std::endl;

        string stats_topic = "~/world_stats";
        this->stat_sub = this->node->Subscribe(
            stats_topic, &CarControlPlugin::OnStats, this);
        std::cerr << "Subscribed world_stats" << std::endl;

        string pose_topic = "~/" + model_name + "/pose";
        this->pose_pub = this->node->Advertise<msgs::PoseStamped>(pose_topic);
        std::cerr << "Advertised pose" << std::endl;
    }

    void
    OnVelCmd(ConstAnyPtr &msg)
    {
        auto vel = msg->double_value();
        std::cerr << "Got vel cmd: " << vel << std::endl;
        this->SetSpeed(vel);
    }

    void
    OnTurnCmd(ConstAnyPtr &msg)
    {
        auto turn = msg->double_value();
        std::cerr << "Got turn cmd: " << turn << std::endl;
        this->SetTurn(turn);
    }

    msgs::PoseStamped
    make_pose_msg(Time time, Pose3d pose)
    {
        msgs::PoseStamped ps;
        // time (Time)
        auto time_msg = ps.mutable_time();
        time_msg->set_sec(time.sec);
        time_msg->set_nsec(time.nsec);

        // pose (Pose)
        auto pose_msg = ps.mutable_pose();

        // // position (Vector3d)
        auto pos = pose.Pos();
        auto pos_msg = pose_msg->mutable_position();
        pos_msg->set_x(pos.X());
        pos_msg->set_y(pos.Y());
        pos_msg->set_z(pos.Z());

        // // orientation (Quaternion)
        auto ori = pose.Rot();
        auto ori_msg = pose_msg->mutable_orientation();
        ori_msg->set_x(ori.X());
        ori_msg->set_y(ori.Y());
        ori_msg->set_z(ori.Z());
        ori_msg->set_w(ori.W());

        return ps;
    }

    void
    OnStats(ConstAnyPtr &_msg)
    {
        auto pose = this->model->WorldPose();
        auto time = Time::GetWallTime();
        auto msg = make_pose_msg(time, pose);

        //std::cerr << "pose: " << msg.ShortDebugString() << std::endl;
        this->pose_pub->Publish(msg);
    }
};

GZ_REGISTER_MODEL_PLUGIN(CarControlPlugin)

