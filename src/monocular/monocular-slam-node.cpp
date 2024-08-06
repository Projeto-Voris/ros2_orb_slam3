#include "monocular-slam-node.hpp"

#include<opencv2/core/core.hpp>

using std::placeholders::_1;

MonocularSlamNode::MonocularSlamNode(ORB_SLAM3::System *pSLAM)
    : Node("orbslam") {
    m_SLAM = pSLAM;
    // std::cout << "slam changed" << std::endl;
    m_image_subscriber = this->create_subscription<ImageMsg>("/stereo_left", 10,
                                                             std::bind(&MonocularSlamNode::GrabImage, this,
                                                                       std::placeholders::_1));
    publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("pose", 10);
    pclpublisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("pointcloud", 10);
    std::cout << "slam changed" << std::endl;
}

MonocularSlamNode::~MonocularSlamNode() {
    // Stop all threads
    m_SLAM->Shutdown();
}

void MonocularSlamNode::GrabImage(const ImageMsg::SharedPtr msg) {
    // Copy the ros image message to cv::Mat.
    try {
        m_cvImPtr = cv_bridge::toCvCopy(msg);

        cv::resize(m_cvImPtr->image, resize, cv::Size(), 0.5, 0.5);
    } catch (cv_bridge::Exception &e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    Sophus::SE3f SE3 = m_SLAM->TrackMonocular(
        resize, static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9);
    //std::cout<<std::setprecision (15)<<static_cast<double>(msg->header.stamp.sec) + static_cast<double>(msg->header.stamp.nanosec) * 1e-9<<std::endl;
    geometry_msgs::msg::PoseStamped sendmsg = SetPoseStamped(SE3);
    sensor_msgs::msg::PointCloud2 point_cloud2 = SetPointCould(m_SLAM->GetTrackedMapPoints(), SE3);
    publisher->publish(sendmsg);
    pclpublisher->publish(point_cloud2);
    std::cout << "Parameter:" << SE3.angleX() << std::endl;
}

geometry_msgs::msg::PoseStamped MonocularSlamNode::SetPoseStamped(Sophus::SE3f &SE3) {
    auto sendmsg = geometry_msgs::msg::PoseStamped();
    sendmsg.header.stamp = this->get_clock()->now();
    sendmsg.header.frame_id = "map";

    sendmsg.pose.position.x = SE3.params()(4);
    sendmsg.pose.position.y = SE3.params()(5);
    sendmsg.pose.position.z = SE3.params()(6);

    sendmsg.pose.orientation.x = SE3.params()(0);
    sendmsg.pose.orientation.y = SE3.params()(1);
    sendmsg.pose.orientation.z = SE3.params()(2);
    sendmsg.pose.orientation.w = SE3.params()(3);
    return sendmsg;
}


sensor_msgs::msg::PointCloud2 MonocularSlamNode::SetPointCould(std::vector<ORB_SLAM3::MapPoint *> points, Sophus::SE3f& SE3) {
    auto pointcloudmsg = sensor_msgs::msg::PointCloud2();
    std::vector<int> indexes;
    cv_bridge::CvImage img_bridge;
    sensor_msgs::msg::Image img_msg;

        int count =0;
        for (size_t i = 0; i < points.size(); i++)
        {
            if(points[i] != 0){
                count++;
                indexes.push_back(i);

            }
        }

        pointcloudmsg.header.stamp = this->get_clock()->now();
        pointcloudmsg.header.frame_id = "map";
        pointcloudmsg.height = 1;
        pointcloudmsg.width = count;
        pointcloudmsg.is_dense = true;
        pointcloudmsg.fields.resize(3);

        // Populate the fields
        pointcloudmsg.fields[0].name = "x";
        pointcloudmsg.fields[0].offset = 0;
        pointcloudmsg.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloudmsg.fields[0].count = 1;

        pointcloudmsg.fields[1].name = "y";
        pointcloudmsg.fields[1].offset = 4;
        pointcloudmsg.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloudmsg.fields[1].count = 1;

        pointcloudmsg.fields[2].name = "z";
        pointcloudmsg.fields[2].offset = 8;
        pointcloudmsg.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
        pointcloudmsg.fields[2].count = 1;

        pointcloudmsg.point_step = 12; // Size of a single point in bytes (3 floats * 4 bytes/float)
        pointcloudmsg.row_step = pointcloudmsg.point_step * pointcloudmsg.width;
        pointcloudmsg.is_bigendian = true;
        pointcloudmsg.data.resize(pointcloudmsg.point_step*count);

        for (size_t i = 0; i < count; i++)
        {
            float x = points[indexes[i]]->GetWorldPos()(0);
            float y = points[indexes[i]]->GetWorldPos()(2);
            float z = -points[indexes[i]]->GetWorldPos()(1);

            memcpy(&pointcloudmsg.data[i*12], &x, 4);
            memcpy(&pointcloudmsg.data[i*12 + 4], &y, 4);
            memcpy(&pointcloudmsg.data[i*12 + 8], &z, 4);
        }
    return pointcloudmsg;
}
