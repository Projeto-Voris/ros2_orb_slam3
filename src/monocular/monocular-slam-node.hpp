#define __MONOCULAR_SLAM_NODE_HPP__


#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <cv_bridge/cv_bridge.h>

#include"System.h"
#include"Frame.h"
#include "Map.h"
#include "Tracking.h"


class MonocularSlamNode : public rclcpp::Node
{
public:
    MonocularSlamNode(ORB_SLAM3::System* pSLAM);

    ~MonocularSlamNode();

private:
    using ImageMsg = sensor_msgs::msg::Image;
    using PoseMsg = geometry_msgs::msg::PoseStamped;

    void GrabImage(const sensor_msgs::msg::Image::SharedPtr msg);

    sensor_msgs::msg::PointCloud2 SetPointCould(std::vector<ORB_SLAM3::MapPoint*> points, Sophus::SE3f& SE3);
    geometry_msgs::msg::PoseStamped SetPoseStamped(Sophus::SE3f& SE3);
    ORB_SLAM3::System* m_SLAM;

    cv_bridge::CvImagePtr m_cvImPtr;
    cv::Mat resize;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr m_image_subscriber;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pclpublisher;
};

