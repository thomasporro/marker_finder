#include "ros/ros.h"

class FindMarkers{
public:
    FindMarkers();
    ~FindMarkers();

    void start();

private:
    struct Params{
        int rate{1};
        std::string outTopic{"image"};
        std::string inTopic{"/k01/ir/image_rect"};
    }

    Params params;
    ros::NodeHandle nodeHandle;
    ros::Subscriber sub;
    ros::Publisher pub;

    std::string getImageEncoding();
};