/*
 * @Descripttion:
 * @Author: Meng Kai
 * @version:
 * @Date: 2023-07-03 20:08:10
 * @LastEditors: Meng Kai
 * @LastEditTime: 2023-07-03 21:01:35
 */
#pragma once
#include "wrapper/ros_noetic/lidar_process/common_lidar_process_interface.h"
#include <algorithm>

namespace HesaiXT16 {
  struct EIGEN_ALIGN16 Point
  {
      PCL_ADD_POINT4D;
      float intensity;
      double timestamp;
      std::uint16_t ring;
  };  
}

POINT_CLOUD_REGISTER_POINT_STRUCT(HesaiXT16::Point,
  (float,x,x)
  (float,y,y)
  (float,z,z)
  (float,intensity,intensity)
  (double,timestamp,timestamp)
  (std::uint16_t,ring,ring)
)

namespace ROSNoetic
{

    class HesaiXT16Process :public CommonLidarProcessInterface
    {
    private:
           using PointType = HesaiXT16::Point;
    public:
        bool process(const sensor_msgs::PointCloud2 &msg, IESKFSlam::PointCloud &cloud){
            pcl::PointCloud<PointType> rs_cloud;
            pcl::fromROSMsg(msg,rs_cloud);
            cloud.cloud_ptr->clear();

            std::sort(rs_cloud.points.begin(), rs_cloud.points.end(), [](const PointType& point_left, const PointType& point_right){
                return point_left.timestamp < point_right.timestamp;
            });
            
            double start_time = rs_cloud.points.begin()->timestamp;
            double end_time = rs_cloud.points.rbegin()->timestamp;

            IESKFSlam::Point point;
            double point_time;

            std::vector<int> ring_k(1024, 0);

            float blind = 0.5;
            float sqr_blind = blind * blind;

            for(const auto& p : rs_cloud.points)
            {
                
                if(std::isnan(p.x) || std::isnan(p.y) || std::isnan(p.z))
                    continue;

                if(++ring_k[p.ring] % _point_skip) {
                    continue;
                }

                if(p.x * p.x + p.y * p.y + p.z * p.z < sqr_blind) {
                    continue;
                }
                    
                point_time = p.timestamp;
                
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;
                
                point.offset_time = (point_time - start_time)*1e9;
                point.ring = p.ring;
                cloud.cloud_ptr->push_back(point);  
            }
            cloud.time_stamp.fromSec(start_time);
            return true;
        }
    };
}