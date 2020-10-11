#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>

class TestClass
{
    private:
    ros::NodeHandle nh_;
    ros::Subscriber marker_point;
    geometry_msgs::PoseStamped pushButtonPose;

    public:
    TestClass();
    ~TestClass();
    void markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg);
    void update();
};

TestClass::TestClass():
    nh_("")
{
    marker_point = nh_.subscribe("/button_tracker_3d/markers", 10, &TestClass::markerCallback, this);
}

TestClass::~TestClass()
{
    std::cout << "program shutdown..." << std::endl;
}

void TestClass::markerCallback(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    std::cout << msg->markers[0].pose << std::endl;
    pushButtonPose.header = msg->markers[0].header;
    pushButtonPose.pose = msg->markers[0].pose;
    // push_start = true;
    // pushButtonPose.header = msg->markers[0].header;
    // pushButtonPose.pose = msg->markers[0].pose;
    // for (int i = 0; msg->markers.size(); i++)
    // {
    //     if (!msg->markers[i].id)
    //     {
    //         pushButtonPose.header = msg->markers[i].header;
    //         pushButtonPose.pose = msg->markers[i].pose;
    //     }        
    // }
}

void TestClass::update()
{
    // std::cout << "marker pose callback test" << std::endl;
}

void markerCallback_(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
    std::cout << msg->markers[0].pose << std::endl;
    // for (int i = 0; msg->markers.size(); i++)
    // {
    //     if (!msg->markers[i].id)
    //     {
    //         std::cout << msg->markers[i].pose << std::endl;
    //     }        
    // }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "callback_test");
    // ros::NodeHandle nh;
    // ros::Subscriber sub = nh.subscribe("/button_tracker_3d/markers", 10, markerCallback_);

    // ros::Rate loop_rate(10);
    // ros::spin();
    // loop_rate.sleep();
    TestClass tc;

    ros::Rate loop_rate(10);

    // tc.update();
    while (ros::ok())
    {
        tc.update();
        ros::spin();
        loop_rate.sleep();
    }
    // std::cout << "shutdown" << std::endl;
    // ros::spin();
    // loop_rate.sleep();

    return 0;
}