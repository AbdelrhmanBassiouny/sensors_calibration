#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>


class CalibrateCAM {

    private:
    int counter;
    ros::Publisher pub;
    ros::Subscriber number_subscriber;
    image_transport::ImageTransport it;
    image_transport::Subscriber sub;

public:
    CalibrateCAM(ros::NodeHandle *nh): it(*nh) {
        counter = 0;
        pub = nh->advertise<std_msgs::Int64>("/number_count", 10);
        sub = it.subscribe("/usb_cam/image_raw", 1, &CalibrateCAM::img_callback, this);
        cv::namedWindow("Image", cv::WND_PROP_AUTOSIZE);
        // image_transport::Publisher pub = it.advertise("out_image_base_topic", 1);
    }

    void img_callback(const sensor_msgs::ImageConstPtr& msg) {
        // We want to scale floating point images so that they display nicely
        counter += 1;
        bool do_dynamic_scaling = false;
        if (msg->encoding.find("F") != std::string::npos) {
            do_dynamic_scaling = true;
        }
         // Convert to OpenCV native BGR color
        cv_bridge::CvImageConstPtr cv_ptr;
        try {
            cv_bridge::CvtColorForDisplayOptions options;
            options.do_dynamic_scaling = do_dynamic_scaling;
            options.min_image_value = 0;
            if (msg->encoding == "32FC1") {
                options.max_image_value = 10;  // 10 [m]
            } else if (msg->encoding == "16UC1") {
                options.max_image_value = 10 * 1000;  // 10 * 1000 [mm]
            }
            cv_ptr = cvtColorForDisplay(cv_bridge::toCvShare(msg), "", options);
        }
        catch (cv_bridge::Exception& e) {
            ROS_ERROR("Unable to convert '%s' image for display: '%s'",
                                    msg->encoding.c_str(), e.what());
        }
        cv::Mat image(cv_ptr->image.clone());
        cv::imshow("Image", image);
        cv::waitKey(1);
    }
    void destructor(){
        cv::destroyWindow("Image");
    }
};

int main (int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_cam");
    ros::NodeHandle nh;
    CalibrateCAM nc = CalibrateCAM(&nh);
    ros::spin();
    nc.destructor();
}