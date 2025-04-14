/*
 * @Descripttion:
 * @Author: MengKai
 * @version:
 * @Date: 2023-06-09 00:07:58
 * @LastEditors: Danny 986337252@qq.com
 * @LastEditTime: 2023-07-02 15:27:35
 */
#include "ieskf_slam/modules/frontend/frontend.h"
#include "ieskf_slam/math/mean_cov_filter.h"

namespace IESKFSlam {
    FrontEnd::FrontEnd(const std::string &config_file_path, const std::string &prefix)
        : ModuleBase(config_file_path, prefix, "Front End Module") {
        float leaf_size;
        readParam("filter_leaf_size", leaf_size, 0.5f);
        voxel_filter.setLeafSize(leaf_size, leaf_size, leaf_size);
        std::vector<double> extrin_v;
        readParam("extrin_r", extrin_v, std::vector<double>());
        extrin_r.setIdentity();
        extrin_t.setZero();
        if (extrin_v.size() == 9) {
            Eigen::Matrix3d extrin_r33;
            extrin_r33 << extrin_v[0], extrin_v[1], extrin_v[2], extrin_v[3], extrin_v[4],
                extrin_v[5], extrin_v[6], extrin_v[7], extrin_v[8];
            extrin_r = extrin_r33;
        } else if (extrin_v.size() == 3) {
            extrin_r.x() = extrin_v[0];
            extrin_r.y() = extrin_v[1];
            extrin_r.z() = extrin_v[2];
            extrin_r.w() = extrin_v[3];
        }
        readParam("extrin_t", extrin_v, std::vector<double>());
        if (extrin_v.size() == 3) {
            extrin_t << extrin_v[0], extrin_v[1], extrin_v[2];
        }
        ieskf_ptr = std::make_shared<IESKF>(config_file_path, "ieskf");
        map_ptr = std::make_shared<RectMapManager>(config_file_path, "map");

        fbpropagate_ptr = std::make_shared<FrontbackPropagate>();
        lio_zh_model_ptr = std::make_shared<LIOZHModel>();
        ieskf_ptr->calc_zh_ptr = lio_zh_model_ptr;
        filter_point_cloud_ptr = pcl::make_shared<PCLPointCloud>();
        lio_zh_model_ptr->prepare(map_ptr->readKDtree(), filter_point_cloud_ptr,
                                  map_ptr->getLocalMap());

        readParam("enable_record", enable_record, false);
        readParam("record_file_name", record_file_name, std::string("default.txt"));
        if (enable_record) {
            record_file.open(RESULT_DIR + record_file_name, std::ios::out | std::ios::app);
        }

        print_table();
    }
    FrontEnd::~FrontEnd() { record_file.close(); }
    void FrontEnd::addImu(const IMU &imu) { imu_deque.push_back(imu); }
    void FrontEnd::addPointCloud(const PointCloud &pointcloud) {
        pointcloud_deque.push_back(pointcloud);
        pcl::transformPointCloud(*pointcloud_deque.back().cloud_ptr,
                                 *pointcloud_deque.back().cloud_ptr,
                                 compositeTransform(extrin_r, extrin_t).cast<float>());
    }
    bool FrontEnd::track() {
        MeasureGroup mg;
        if (syncMeasureGroup(mg)) {
            if (!imu_inited) {
                map_ptr->reset();
                map_ptr->addScan(mg.cloud.cloud_ptr, Eigen::Quaterniond::Identity(),
                                 Eigen::Vector3d::Zero());
                initState(mg);
                return false;
            }
            fbpropagate_ptr->propagate(mg, ieskf_ptr);
            voxel_filter.setInputCloud(mg.cloud.cloud_ptr);
            voxel_filter.filter(*filter_point_cloud_ptr);
            if(!ieskf_ptr->update())
                return false;
            auto state = ieskf_ptr->getX();
            if (enable_record) {
                record_file << std::setprecision(15) << mg.lidar_end_time << " "
                            << state.position.x() << " " << state.position.y() << " "
                            << state.position.z() << " " << state.rotation.x() << " "
                            << state.rotation.y() << " " << state.rotation.z() << " "
                            << state.rotation.w() << std::endl;
            }
            map_ptr->addScan(filter_point_cloud_ptr, state.rotation, state.position);
            return true;
        }
        return false;
    }
    const PCLPointCloud &FrontEnd::readCurrentPointCloud() { return *filter_point_cloud_ptr; }
    const PCLPointCloud &FrontEnd::readCurrentLocalMap() { return *map_ptr->getLocalMap(); }
    bool FrontEnd::syncMeasureGroup(MeasureGroup &mg) {
        mg.imus.clear();
        mg.cloud.cloud_ptr->clear();
        if (pointcloud_deque.empty() || imu_deque.empty()) {
            return false;
        }
        ///. wait for imu
        double imu_end_time = imu_deque.back().time_stamp.sec();
        double imu_start_time = imu_deque.front().time_stamp.sec();
        double cloud_start_time = pointcloud_deque.front().time_stamp.sec();
        double cloud_end_time =
            pointcloud_deque.front().cloud_ptr->points.back().offset_time / 1e9 + cloud_start_time;

        if (imu_end_time < cloud_end_time) {
            return false;
        }

        if (cloud_end_time < imu_start_time) {
            pointcloud_deque.pop_front();
            return false;
        }
        mg.cloud = pointcloud_deque.front();
        pointcloud_deque.pop_front();
        mg.lidar_begin_time = cloud_start_time;
        mg.lidar_end_time = cloud_end_time;
        while (!imu_deque.empty()) {
            if (imu_deque.front().time_stamp.sec() < mg.lidar_end_time) {
                mg.imus.push_back(imu_deque.front());
                imu_deque.pop_front();

            } else {
                break;
            }
        }
        if (mg.imus.size() <= 5) {
            return false;
        }
        return true;
    }

    void FrontEnd::initState(MeasureGroup &mg) {

        mpcdps::MeanCovFilter<3> mcf_acc, mcf_gyr;
        for (size_t i = 0; i < mg.imus.size(); i++) {
            mcf_acc.push(mg.imus[i].acceleration);
            mcf_gyr.push(mg.imus[i].gyroscope);
        }

        const Eigen::Vector3d acc_mean = mcf_acc.get_mean();
        const Eigen::Matrix3d acc_cov = mcf_acc.get_covariance();
        const Eigen::Vector3d gyr_mean = mcf_gyr.get_mean();
        const Eigen::Matrix3d gyr_cov = mcf_gyr.get_covariance();

        std::cout<<"acc_mean: "<<acc_mean.transpose()<<std::endl;
        std::cout<<"gyr_mean: "<<gyr_mean.transpose()<<std::endl;

        double acc_norm = acc_mean.norm();
        double gyr_norm = gyr_mean.norm();

        if (std::abs(acc_norm - 9.8) > 1 || gyr_norm > 1) {
            return;
        }        

        double imu_scale = GRAVITY / acc_norm;

        Eigen::Vector3d gI = acc_mean;
        gI.normalize();
        Eigen::Vector3d g(0, 0, 1);
        Eigen::Vector3d n = gI.cross(g);  //from IMU to world
        n.normalize();
        double theta = std::acos(g.dot(gI));

        Eigen::Matrix3d R = so3Exp(n * theta);
        std::cout<<"R_wi: \n"<<R<<std::endl;

        auto X = ieskf_ptr->getX();

        X.rotation = Eigen::Quaterniond(R);
        X.gravity << 0, 0, -GRAVITY;
        X.ba = acc_mean + X.rotation.inverse() * X.gravity;
        X.bg = gyr_mean;

        std::cout<<"init, ba="<<X.ba.transpose()<<", bg="<<X.bg.transpose()<<std::endl;

        imu_inited = true;
        fbpropagate_ptr->imu_scale = imu_scale;
        fbpropagate_ptr->last_imu = mg.imus.back();
        fbpropagate_ptr->last_lidar_end_time_ = mg.lidar_end_time;

        ieskf_ptr->setX(X);
    }
    
    IESKF::State18 FrontEnd::readState() { return ieskf_ptr->getX(); }
}  // namespace IESKFSlam
