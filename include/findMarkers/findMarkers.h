#include "ros/ros.h"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "image_transport/image_transport.h"


class FindMarkers{
public:
    FindMarkers(){};
    ~FindMarkers(){};

    void start();

private:
    struct Params{
        int rate{1000};
        std::string outTopic{"image"};
        std::string inTopic{"/k01/ir/image_rect"};
    };

    Params params;
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string getImageEncoding();
    std::tuple<std::vector<std::vector<cv::Point>>, std::vector<cv::Point>> findCenters(cv::Mat image);
    void listenerCallback(const sensor_msgs::ImageConstPtr& Image);
};
