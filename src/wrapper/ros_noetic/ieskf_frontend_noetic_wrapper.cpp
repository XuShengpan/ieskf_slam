/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-08 21:05:55
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:25:59
 */
#include "wrapper/ros_noetic/ieskf_frontend_noetic_wrapper.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace ROSNoetic {

    IESKFFrontEndWrapper::IESKFFrontEndWrapper(ros::NodeHandle &nh) {

        std::string config_file_name, lidar_topic, imu_topic;
        nh.param<std::string>("wrapper/config_file_name", config_file_name, "");
        nh.param<std::string>("wrapper/lidar_topic", lidar_topic, "/lidar");
        nh.param<std::string>("wrapper/imu_topic", imu_topic, "/imu");

        front_end_ptr =
            std::make_shared<IESKFSlam::FrontEnd>(CONFIG_DIR + config_file_name, "front_end");

        // 发布者和订阅者
        cloud_subscriber =
            nh.subscribe(lidar_topic, 100, &IESKFFrontEndWrapper::lidarCloudMsgCallBack, this);
        imu_subscriber = nh.subscribe(imu_topic, 100, &IESKFFrontEndWrapper::imuMsgCallBack, this);
        // 读取雷达类型
        int lidar_type = 0, point_skip = 4;
        nh.param<int>("wrapper/lidar_type", lidar_type, AVIA);
        nh.param<int>("wrapper/point_skip", point_skip, 4);

        if (lidar_type == AVIA) {
            lidar_process_ptr = std::make_shared<AVIAProcess>();
        } else if (lidar_type == VELO) {
            lidar_process_ptr = std::make_shared<VelodyneProcess>();
        } else if(lidar_type == HESAI_XT16) {
            lidar_process_ptr = std::make_shared<HesaiXT16Process>();
        } else {
            std::cout << "unsupport lidar type" << std::endl;
            exit(100);
        }

        lidar_process_ptr->set_point_skip(point_skip);

        nh.param<std::string>("publish/odometry_topic", odom_topic, "/odom");
        nh.param<std::string>("publish/global_point_cloud_topic", global_cloud_topic, "local_map");
        nh.param<std::string>("publish/body_point_cloud_topic", body_cloud_topic, "curr_cloud");
        nh.param<std::string>("publish/path_topic", path_topic, "path");
        nh.param<std::string>("publish/world_frame_id", world_frame_id, "world");
        nh.param<std::string>("publish/body_frame_id", body_frame_id, "body");
        nh.param<bool>("publish/publish_lidar_scan_in_local_frame", 
            publish_lidar_scan_in_local_frame,  true);

        curr_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(body_cloud_topic, 100);
        path_pub = nh.advertise<nav_msgs::Path>(path_topic, 100);
        local_map_pub = nh.advertise<sensor_msgs::PointCloud2>(global_cloud_topic, 100);
        odom_pub = nh.advertise<nav_msgs::Odometry>(odom_topic, 100);

        run();
    }

    IESKFFrontEndWrapper::~IESKFFrontEndWrapper() {}
    void IESKFFrontEndWrapper::lidarCloudMsgCallBack(const sensor_msgs::PointCloud2Ptr &msg) {
        IESKFSlam::PointCloud cloud;
        lidar_process_ptr->process(*msg, cloud);
        front_end_ptr->addPointCloud(cloud);
    }

    void IESKFFrontEndWrapper::imuMsgCallBack(const sensor_msgs::ImuPtr &msg) {
        IESKFSlam::IMU imu;
        imu.time_stamp.fromNsec(msg->header.stamp.toNSec());
        imu.acceleration = {msg->linear_acceleration.x, msg->linear_acceleration.y,
                            msg->linear_acceleration.z};
        imu.gyroscope = {msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z};
        front_end_ptr->addImu(imu);
    }
    void IESKFFrontEndWrapper::run() {
        ros::Rate rate(500);
        while (ros::ok()) {
            rate.sleep();
            ros::spinOnce();
            if (front_end_ptr->track()) {
                publishMsg();
            }
        }
    }
    void IESKFFrontEndWrapper::publishMsg() {
        static nav_msgs::Path path;
        nav_msgs::Odometry odom;

        auto X = front_end_ptr->readState();
        const Eigen::Vector3d& p = X.position;
        const Eigen::Quaterniond& q = X.rotation;
        auto stamp = ros::Time().fromSec(X.time);

        static tf::TransformBroadcaster br;
        tf::Transform                   transform;
        transform.setOrigin(tf::Vector3(p[0], p[1], p[2]));
        transform.setRotation( tf::Quaternion(q.x(), q.y(), q.z(), q.w()));
        br.sendTransform( tf::StampedTransform( transform, stamp, world_frame_id, body_frame_id) );

        odom.header.frame_id = world_frame_id;
        odom.child_frame_id = body_frame_id;
        odom.header.stamp = stamp;
        odom.pose.pose.position.x = p[0];
        odom.pose.pose.position.y = p[1];
        odom.pose.pose.position.z = p[2];

        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom_pub.publish(odom);

        path.header.frame_id = world_frame_id;
        path.header.stamp = stamp;

        geometry_msgs::PoseStamped psd;
        psd.pose.position.x = X.position.x();
        psd.pose.position.y = X.position.y();
        psd.pose.position.z = X.position.z();
        path.poses.push_back(psd);
        path_pub.publish(path);
        
        bool init_map = false;
        sensor_msgs::PointCloud2 msg;
        IESKFSlam::PCLPointCloud cloud = front_end_ptr->readCurrentPointCloud();
        if(!publish_lidar_scan_in_local_frame || !init_map) {
            pcl::transformPointCloud(cloud, cloud, IESKFSlam::compositeTransform(X.rotation, X.position).cast<float>());
            pcl::toROSMsg(cloud, msg);
            msg.header.frame_id = world_frame_id;
            init_map = true;
        } else {
            pcl::toROSMsg(cloud, msg);
            msg.header.frame_id = body_frame_id;
        }

        msg.header.stamp = stamp;
        curr_cloud_pub.publish(msg);

        cloud = front_end_ptr->readCurrentLocalMap();
        pcl::toROSMsg(cloud, msg);
        msg.header.frame_id = world_frame_id;
        msg.header.stamp = stamp;
        local_map_pub.publish(msg);

    }
}  // namespace ROSNoetic
