#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>

class MultiImageConverter
{
public:
    MultiImageConverter() : nh_("~")
    {
        // 初始化4路图像转换器
        initConverter(1);
        initConverter(2);
        initConverter(3);
        initConverter(4);
    }

private : void
          initConverter(int id)
{
    // 创建订阅者
    std::string sub_topic = "/m_gmsl_" + std::to_string(id) + "/image_compressed";
    std::string pub_topic = "/m_gmsl_" + std::to_string(id) + "/image_raw";
    // 创建发布者
    publishers_.push_back(nh_.advertise<sensor_msgs::Image>(pub_topic, 1));
    subscribers_.push_back(nh_.subscribe<sensor_msgs::CompressedImage>(
        sub_topic, 1,
        boost::bind(&MultiImageConverter::imageCallback, this, _1, id)));
}

void imageCallback(const sensor_msgs::CompressedImageConstPtr &msg, int id)
{
    try
    {
        // 将JPEG数据解码为OpenCV格式
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(
            msg,
            sensor_msgs::image_encodings::BGR8);
        // 创建ROS Image消息
        sensor_msgs::ImagePtr out_msg = cv_ptr->toImageMsg();
        // 发布转换后的消息
        publishers_[id - 1].publish(out_msg);
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("Camera%d: cv_bridge exception: %s", id, e.what());
    }
    catch (const std::exception &e)
    {
        ROS_ERROR("Camera%d: Decoding error: %s", id, e.what());
    }
}
ros::NodeHandle nh_;
std::vector<ros::Subscriber> subscribers_;
std::vector<ros::Publisher> publishers_;
}
;
