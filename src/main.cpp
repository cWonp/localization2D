#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/registration/icp.h>

#include <cmath>

using namespace std;

class PoseEstimate
{
private:
    ros::NodeHandle nh;

    ros::Subscriber subScan;
    ros::Subscriber subOdom;

    ros::Publisher pub2DPCloud;
    ros::Publisher pub2DLaserScan;

    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloudCroped2D;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scanCloudprev;
    std_msgs::Header cloudHeader;

    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    bool firstScan = true;

public:
    PoseEstimate()
    {
        subScan = nh.subscribe<sensor_msgs::PointCloud2>("/scan", 2, &PoseEstimate::scanCallback, this);
        subOdom = nh.subscribe<nav_msgs::Odometry>("/odom", 2, &PoseEstimate::odomCallback, this);

        pub2DPCloud = nh.advertise<sensor_msgs::PointCloud2>("/pcloud2d", 2);
        pub2DLaserScan = nh.advertise<sensor_msgs::LaserScan>("/laserscan", 2);

        //Setting ICP
        icp.setMaxCorrespondenceDistance(1.0);
        icp.setTransformationEpsilon(0.001);
        icp.setMaximumIterations(1000);

        allocateMemory();
    }

    void allocateMemory()
    {
        scanCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        scanCloudCroped2D.reset(new pcl::PointCloud<pcl::PointXYZ>());
        scanCloudprev.reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    void resetParameters()
    {
        scanCloud->clear();
        scanCloudCroped2D->clear();
    }

    void scanCallback(const sensor_msgs::PointCloud2::ConstPtr &scanMsg)
    {
        // convert rosmsg to pcl pointcloud
        rosmsg2Pcloud(scanMsg);
        // Obtain 2d pointcloud
        cropPointCloud();
        if(firstScan){
            *scanCloudprev = *scanCloudCroped2D;
            firstScan = false;
        }

        static int cntIcp = 0;
        if((cntIcp++) % 5 == 0){
            Eigen::Matrix4f result;
            result = runICP();
        }
        // Convert to laserscan rosmsg

        // pub rosmsg
        resetParameters();
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg)
    {
    }

    Eigen::Matrix4f runICP()
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr align(new pcl::PointCloud<pcl::PointXYZ>);      
        icp.setInputSource(scanCloudprev);
        icp.setInputTarget(scanCloudCroped2D);
        icp.align(*align);

        Eigen::Matrix4f src2tgt = icp.getFinalTransformation();
        double score = icp.getFitnessScore();
        bool is_converged = icp.hasConverged();

        cout<<"score = "<<score<<endl;
        cout<<"is_converged =  "<<is_converged<<endl;
        *scanCloudprev = *scanCloudCroped2D;

        return src2tgt;
    }

    void cropPointCloud()
    {
        pcl::CropBox<pcl::PointXYZ> boxFilter;
        boxFilter.setMin(Eigen::Vector4f(-8.0f, -8.0f, 0.30f, 1.0));
        boxFilter.setMax(Eigen::Vector4f(8.0f, 8.0f, 0.35f, 1.0));
        boxFilter.setInputCloud(scanCloud);
        boxFilter.filter(*scanCloudCroped2D);
    }

    // convert data type
    void rosmsg2Pcloud(const sensor_msgs::PointCloud2ConstPtr &scanMsg)
    {
        cloudHeader = scanMsg->header;
        pcl::fromROSMsg(*scanMsg, *scanCloud);
    }
    void pcloud2Rosmsg()
    {
        sensor_msgs::PointCloud2 msgScan2D;
        pcl::toROSMsg(*scanCloudCroped2D, msgScan2D);
        msgScan2D.header = cloudHeader;
        pub2DPCloud.publish(msgScan2D);
        pcloud2Laserscan(msgScan2D);
    }
    void pcloud2Laserscan(const sensor_msgs::PointCloud2& cloud_msg)
    {
        sensor_msgs::LaserScan msgLaserscan;
        sensor_msgs::PointCloud2 cloud_out;
        msgLaserscan.header = cloudHeader;
        msgLaserscan.angle_min = -M_PI;
        msgLaserscan.angle_max = M_PI;
        msgLaserscan.angle_increment = M_PI / 180.0;
        msgLaserscan.time_increment = 0.0;
        msgLaserscan.scan_time = 1.0 / 30.0;
        msgLaserscan.range_min = 0;
        msgLaserscan.range_max = std::numeric_limits<double>::max();
        double inf_epsilon_ = 1.0;

        uint32_t ranges_size = std::ceil((msgLaserscan.angle_max - msgLaserscan.angle_min) / msgLaserscan.angle_increment);
        msgLaserscan.ranges.assign(ranges_size, msgLaserscan.range_max + inf_epsilon_);
        
        cloud_out = cloud_msg;

        for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(cloud_out, "x"), iter_y(cloud_out, "y"),
             iter_z(cloud_out, "z");
             iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z)
        {
            double range = hypot(*iter_x, *iter_y);
            double angle = atan2(*iter_y, *iter_z);
            int inedx = (angle - msgLaserscan.angle_min) / msgLaserscan.angle_increment;
            msgLaserscan.ranges.push_back(range);
        }

        pub2DLaserScan.publish(msgLaserscan);
    }

    ~PoseEstimate() {}
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "icp_2d");

    PoseEstimate run;

    ros::spin();
    return 0;
}
