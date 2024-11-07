#include <rclcpp/rclcpp.hpp>  
#include <sensor_msgs/msg/point_cloud2.hpp>  
#include <pcl/point_cloud.h>  
#include <pcl/point_types.h>  
#include <pcl_conversions/pcl_conversions.h>  
  
struct PointXYZIRT  
{  
    PCL_ADD_POINT4D;  
    PCL_ADD_INTENSITY;  
    uint16_t ring;  
    float time;  
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
} EIGEN_ALIGN16;  
  
POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRT,  
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(float, time, time))  
  
class LidarDataNode : public rclcpp::Node  
{  
public:  
    LidarDataNode() : Node("lidar_data")  
    {  
        // 订阅输入点云  
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(  
            "/laser3d/pointcloud",  
            10,  
            std::bind(&LidarDataNode::cloudCallback, this, std::placeholders::_1));  
    }  
  
private:  
    void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr laserCloudIn)  
    {  
        pcl::PointCloud<PointXYZIRT>::Ptr cloud_filtered(new pcl::PointCloud<PointXYZIRT>);  
        pcl::fromROSMsg(*laserCloudIn, *cloud_filtered);  
  
        for (size_t i = 0; i < cloud_filtered->points.size(); ++i)  
        {  
            const PointXYZIRT& point = cloud_filtered->points[i];  
            RCLCPP_INFO(this->get_logger(), "x: %f y: %f z: %f", point.x, point.y, point.z);  
        }  
    }  
  
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;  
};  
  
int main(int argc, char **argv)  
{  
    rclcpp::init(argc, argv);  
    rclcpp::spin(std::make_shared<LidarDataNode>());  
    rclcpp::shutdown();  
    return 0;  
}