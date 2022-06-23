#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <sensor_msgs/Image.h>


class CalibrateCAM {

    private:
    int counter;
    ros::Publisher pub;
    ros::Subscriber number_subscriber;

    public:
    CalibrateCAM(ros::NodeHandle *nh) {
        counter = 0;

        pub = nh->advertise<std_msgs::Int64>("/number_count", 10);    
        number_subscriber = nh->subscribe("/usb_cam/image_raw", 10, 
            &CalibrateCAM::img_callback, this);
    }

    void img_callback(const sensor_msgs::Image& msg) {
        counter += 1;
        std_msgs::Int64 new_msg;
        new_msg.data = counter;
        pub.publish(new_msg);
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_cam");
    ros::NodeHandle nh;
    CalibrateCAM nc = CalibrateCAM(&nh);
    ros::spin();
}