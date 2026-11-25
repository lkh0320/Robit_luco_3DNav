#include <mutex>
#include <vector>
#include <queue>
#include <memory>
#include <iostream>
#include <chrono>
// #include <filesystem>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
// #include <livox_ros_driver2/msg/custom_msg.hpp>

#include "utils.h"
#include "map_builder/commons.h"
#include "map_builder/map_builder.h"

#include <pcl_conversions/pcl_conversions.h>
#include "tf2_ros/transform_broadcaster.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <yaml-cpp/yaml.h>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;
struct NodeConfig
{
    std::string imu_topic = "/sync_imu";
    std::string lidar_topic = "/sync_velo";
    std::string body_frame = "body";
    std::string world_frame = "lidar";
    bool print_time_cost = false;
    // body_frame(base_link) 기준 IMU 자세: p_body = r_bi * p_imu + t_bi
    // 설정이 없으면 단위변환 -> body_frame == IMU (원본 동작)
    M3D r_bi = M3D::Identity();
    V3D t_bi = V3D::Zero();
    // 휠 오도메트리 연동: 비어 있으면 world_frame -> body_frame 을 직접 발행(원본 동작).
    // 지정하면(예: odom) 휠 오도메트리의 odom_frame -> body_frame TF를 받아
    // world_frame -> odom_frame 을 발행 (map->odom 보정 방식, 휠 odom TF와 충돌 없음)
    std::string odom_frame = "";
    // 라이다 점별 시간으로 스캔 왜곡 보정 (time 필드 필요). false면 원본처럼 보정 없음
    bool lidar_deskew = false;
    // 라이다 시각 보정(초): 라이다 시각 + lidar_time_offset = IMU 시계 기준 시각
    double lidar_time_offset = 0.0;
};
struct StateData
{
    bool lidar_pushed = false;
    std::mutex imu_mutex;
    std::mutex lidar_mutex;
    double last_lidar_time = -1.0;
    double last_imu_time = -1.0;
    std::deque<IMUData> imu_buffer;
    std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>> lidar_buffer;
    nav_msgs::msg::Path path;
};

class LIONode : public rclcpp::Node
{
public:
    LIONode() : Node("lio_node")
    {
        RCLCPP_INFO(this->get_logger(), "LIO Node Started");
        loadParameters();

        m_imu_sub = this->create_subscription<sensor_msgs::msg::Imu>(m_node_config.imu_topic, 10, std::bind(&LIONode::imuCB, this, std::placeholders::_1));
        m_lidar_sub = this->create_subscription<sensor_msgs::msg::PointCloud2>(m_node_config.lidar_topic, 10, std::bind(&LIONode::lidarCB, this, std::placeholders::_1));

        m_body_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("body_cloud", 10000);
        m_world_cloud_pub = this->create_publisher<sensor_msgs::msg::PointCloud2>("world_cloud", 10000);
        m_path_pub = this->create_publisher<nav_msgs::msg::Path>("lio_path", 10000);
        m_odom_pub = this->create_publisher<nav_msgs::msg::Odometry>("lio_odom", 10000);
        m_tf_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
        if (!m_node_config.odom_frame.empty())
        {
            m_tf_buffer = std::make_shared<tf2_ros::Buffer>(this->get_clock());
            m_tf_listener = std::make_shared<tf2_ros::TransformListener>(*m_tf_buffer);
            RCLCPP_INFO(this->get_logger(), "wheel odom mode: publish %s -> %s (uses %s -> %s)",
                        m_node_config.world_frame.c_str(), m_node_config.odom_frame.c_str(),
                        m_node_config.odom_frame.c_str(), m_node_config.body_frame.c_str());
        }

        m_state_data.path.poses.clear();
        m_state_data.path.header.frame_id = m_node_config.world_frame;

        m_kf = std::make_shared<IESKF>();
        m_builder = std::make_shared<MapBuilder>(m_builder_config, m_kf);
        m_timer = this->create_wall_timer(20ms, std::bind(&LIONode::timerCB, this));
    }

    void loadParameters()
    {
        this->declare_parameter("config_path", "");
        // launch에서 덮어쓰기용 (yaml의 odom_frame보다 우선)
        this->declare_parameter("odom_frame", std::string("__yaml__"));
        std::string config_path;
        this->get_parameter<std::string>("config_path", config_path);

        YAML::Node config = YAML::LoadFile(config_path);
        if (!config)
        {
            RCLCPP_WARN(this->get_logger(), "FAIL TO LOAD YAML FILE!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "LOAD FROM YAML CONFIG PATH: %s", config_path.c_str());

        m_node_config.imu_topic = config["imu_topic"].as<std::string>();
        m_node_config.lidar_topic = config["lidar_topic"].as<std::string>();
        m_node_config.body_frame = config["body_frame"].as<std::string>();
        m_node_config.world_frame = config["world_frame"].as<std::string>();
        m_node_config.print_time_cost = config["print_time_cost"].as<bool>();
        if (config["odom_frame"])
            m_node_config.odom_frame = config["odom_frame"].as<std::string>();
        std::string odom_override = this->get_parameter("odom_frame").as_string();
        if (odom_override != "__yaml__")
            m_node_config.odom_frame = odom_override;
        if (config["lidar_deskew"])
            m_node_config.lidar_deskew = config["lidar_deskew"].as<bool>();
        if (config["lidar_time_offset"])
            m_node_config.lidar_time_offset = config["lidar_time_offset"].as<double>();
        RCLCPP_INFO(this->get_logger(), "lidar_deskew=%d lidar_time_offset=%.4f",
                    m_node_config.lidar_deskew, m_node_config.lidar_time_offset);
        if (config["t_bi"] && config["r_bi"])
        {
            std::vector<double> t_bi_vec = config["t_bi"].as<std::vector<double>>();
            std::vector<double> r_bi_vec = config["r_bi"].as<std::vector<double>>();
            m_node_config.t_bi << t_bi_vec[0], t_bi_vec[1], t_bi_vec[2];
            m_node_config.r_bi << r_bi_vec[0], r_bi_vec[1], r_bi_vec[2], r_bi_vec[3], r_bi_vec[4], r_bi_vec[5], r_bi_vec[6], r_bi_vec[7], r_bi_vec[8];
            RCLCPP_INFO(this->get_logger(), "body->imu extrinsic t_bi: [%.4f, %.4f, %.4f]",
                        m_node_config.t_bi.x(), m_node_config.t_bi.y(), m_node_config.t_bi.z());
        }

        m_builder_config.lidar_filter_num = config["lidar_filter_num"].as<int>();
        m_builder_config.lidar_min_range = config["lidar_min_range"].as<double>();
        m_builder_config.lidar_max_range = config["lidar_max_range"].as<double>();
        m_builder_config.scan_resolution = config["scan_resolution"].as<double>();
        m_builder_config.map_resolution = config["map_resolution"].as<double>();
        m_builder_config.cube_len = config["cube_len"].as<double>();
        m_builder_config.det_range = config["det_range"].as<double>();
        m_builder_config.move_thresh = config["move_thresh"].as<double>();
        m_builder_config.na = config["na"].as<double>();
        m_builder_config.ng = config["ng"].as<double>();
        m_builder_config.nba = config["nba"].as<double>();
        m_builder_config.nbg = config["nbg"].as<double>();

        m_builder_config.imu_init_num = config["imu_init_num"].as<int>();
        m_builder_config.near_search_num = config["near_search_num"].as<int>();
        m_builder_config.ieskf_max_iter = config["ieskf_max_iter"].as<int>();
        m_builder_config.gravity_align = config["gravity_align"].as<bool>();
        m_builder_config.esti_il = config["esti_il"].as<bool>();
        std::vector<double> t_il_vec = config["t_il"].as<std::vector<double>>();
        std::vector<double> r_il_vec = config["r_il"].as<std::vector<double>>();
        m_builder_config.t_il << t_il_vec[0], t_il_vec[1], t_il_vec[2];
        m_builder_config.r_il << r_il_vec[0], r_il_vec[1], r_il_vec[2], r_il_vec[3], r_il_vec[4], r_il_vec[5], r_il_vec[6], r_il_vec[7], r_il_vec[8];
        m_builder_config.lidar_cov_inv = config["lidar_cov_inv"].as<double>();
        if (config["degen_enable"]) m_builder_config.degen_enable = config["degen_enable"].as<bool>();
        if (config["degen_ratio"]) m_builder_config.degen_ratio = config["degen_ratio"].as<double>();
        if (config["degen_min_weight"]) m_builder_config.degen_min_weight = config["degen_min_weight"].as<double>();
        if (config["degen_blocks"]) m_builder_config.degen_blocks = config["degen_blocks"].as<int>();
        if (config["adapt_log"]) m_builder_config.adapt_log = config["adapt_log"].as<bool>();
        RCLCPP_INFO(this->get_logger(), "degen_enable=%d degen_ratio=%.3f degen_blocks=%d",
                    m_builder_config.degen_enable, m_builder_config.degen_ratio, m_builder_config.degen_blocks);
    }

    void imuCB(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(m_state_data.imu_mutex);
        double timestamp = Utils::getSec(msg->header);
        if (timestamp < m_state_data.last_imu_time)
        {
            RCLCPP_WARN(this->get_logger(), "IMU Message is out of order");
            std::deque<IMUData>().swap(m_state_data.imu_buffer);
        }
        m_state_data.imu_buffer.emplace_back(V3D(msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z),
                                             V3D(msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z),
                                             timestamp);
        m_state_data.last_imu_time = timestamp;
    }
    void lidarCB(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {

        // pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZI>);
        // pcl::fromROSMsg(*msg, *cloud_xyz);
        // pcl::NormalEstimation<pcl::PointXYZI, pcl::Normal> ne;
        // ne.setInputCloud(cloud_xyz);
        // auto tree = std::make_shared<pcl::search::KdTree<pcl::PointXYZI>>();
        // ne.setSearchMethod(tree);
        // ne.setKSearch(10);
        // pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
        // ne.compute(*normals);
        // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud_with_normals(new pcl::PointCloud<pcl::PointXYZINormal>);
        // pcl::concatenateFields(*cloud_xyz, *normals, *cloud_with_normals);

        // pcl::PointCloud<pcl::PointXYZINormal>::Ptr cloud = cloud_with_normals;
        // msg->fields[5].name="curvature";
        // CloudType::Ptr cloud(new pcl::PointCloud<pcl::PointXYZINormal>);
        // pcl::fromROSMsg(*msg, *cloud);
        double start_offset = 0.0;
        CloudType::Ptr cloud = Utils::velo2PCL(msg, m_builder_config.lidar_filter_num, m_builder_config.lidar_min_range, m_builder_config.lidar_max_range,
                                               m_node_config.lidar_deskew, &start_offset);
        std::lock_guard<std::mutex> lock(m_state_data.lidar_mutex);
        // 스캔 시작 시각 (점별 시간 curvature는 이 시각 기준 ms)
        double timestamp = Utils::getSec(msg->header) + start_offset + m_node_config.lidar_time_offset;
        if (timestamp < m_state_data.last_lidar_time)
        {
            RCLCPP_WARN(this->get_logger(), "Lidar Message is out of order");
            std::deque<std::pair<double, pcl::PointCloud<pcl::PointXYZINormal>::Ptr>>().swap(m_state_data.lidar_buffer);
        }
        m_state_data.lidar_buffer.emplace_back(timestamp, cloud);
        m_state_data.last_lidar_time = timestamp;
    }

    bool syncPackage()
    {
        if (m_state_data.imu_buffer.empty() || m_state_data.lidar_buffer.empty())
            return false;
        if (!m_state_data.lidar_pushed)
        {
            m_package.cloud = m_state_data.lidar_buffer.front().second;
            std::sort(m_package.cloud->points.begin(), m_package.cloud->points.end(), [](PointType &p1, PointType &p2)
                      { return p1.curvature < p2.curvature; });
            m_package.cloud_start_time = m_state_data.lidar_buffer.front().first;
            m_package.cloud_end_time = m_package.cloud_start_time + m_package.cloud->points.back().curvature / 1000.0;
            m_state_data.lidar_pushed = true;
        }
        if (m_state_data.last_imu_time < m_package.cloud_end_time)
            return false;

        Vec<IMUData>().swap(m_package.imus);
        while (!m_state_data.imu_buffer.empty() && m_state_data.imu_buffer.front().time < m_package.cloud_end_time)
        {
            m_package.imus.emplace_back(m_state_data.imu_buffer.front());
            m_state_data.imu_buffer.pop_front();
        }
        m_state_data.lidar_buffer.pop_front();
        m_state_data.lidar_pushed = false;
        return true;
    }

    void publishCloud(rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub, CloudType::Ptr cloud, std::string frame_id, const double &time)
    {
        if (pub->get_subscription_count() <= 0)
            return;
        sensor_msgs::msg::PointCloud2 cloud_msg;
        pcl::toROSMsg(*cloud, cloud_msg);
        cloud_msg.header.frame_id = frame_id;
        cloud_msg.header.stamp = Utils::getTime(time);
        pub->publish(cloud_msg);
    }

    // world_frame 기준 body_frame 자세: T_ob = T_bi * T_wi * T_bi^-1
    // (odom 원점 = 초기화 시점의 body_frame 위치, 축은 IMU world와 같은 중력 정렬)
    void bodyPose(M3D &r_ob, V3D &t_ob)
    {
        const M3D &r_bi = m_node_config.r_bi;
        const V3D &t_bi = m_node_config.t_bi;
        const M3D &r_wi = m_kf->x().r_wi;
        r_ob = r_bi * r_wi * r_bi.transpose();
        t_ob = r_bi * (r_wi * (-r_bi.transpose() * t_bi) + m_kf->x().t_wi) + t_bi;
    }

    void publishOdometry(rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub, std::string frame_id, std::string child_frame, const double &time)
    {
        if (odom_pub->get_subscription_count() <= 0)
            return;
        M3D r_ob;
        V3D t_ob;
        bodyPose(r_ob, t_ob);
        nav_msgs::msg::Odometry odom;
        odom.header.frame_id = frame_id;
        odom.header.stamp = Utils::getTime(time);
        odom.child_frame_id = child_frame;
        odom.pose.pose.position.x = t_ob.x();
        odom.pose.pose.position.y = t_ob.y();
        odom.pose.pose.position.z = t_ob.z();
        Eigen::Quaterniond q(r_ob);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        // body_frame 원점의 선속도(body 좌표): R_bi * (R_wi^T v + w x t_ib)
        V3D omega = V3D::Zero();
        if (!m_package.imus.empty())
            omega = m_package.imus.back().gyro - m_kf->x().bg;
        V3D t_ib = -m_node_config.r_bi.transpose() * m_node_config.t_bi;
        V3D vel = m_node_config.r_bi * (m_kf->x().r_wi.transpose() * m_kf->x().v + omega.cross(t_ib));
        odom.twist.twist.linear.x = vel.x();
        odom.twist.twist.linear.y = vel.y();
        odom.twist.twist.linear.z = vel.z();
        V3D ang = m_node_config.r_bi * omega;
        odom.twist.twist.angular.x = ang.x();
        odom.twist.twist.angular.y = ang.y();
        odom.twist.twist.angular.z = ang.z();
        odom_pub->publish(odom);
    }

    void publishPath(rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub, std::string frame_id, const double &time)
    {
        if (path_pub->get_subscription_count() <= 0)
            return;
        M3D r_ob;
        V3D t_ob;
        bodyPose(r_ob, t_ob);
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = Utils::getTime(time);
        pose.pose.position.x = t_ob.x();
        pose.pose.position.y = t_ob.y();
        pose.pose.position.z = t_ob.z();
        Eigen::Quaterniond q(r_ob);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        m_state_data.path.poses.push_back(pose);
        path_pub->publish(m_state_data.path);
    }

    // world_frame -> odom_frame = T_world_body * (T_odom_body)^-1
    //   T_world_body: LIO 추정, T_odom_body: 휠 오도메트리 TF (같은 시각으로 조회)
    void broadCastWorldToOdom(const double &time)
    {
        geometry_msgs::msg::TransformStamped ob;
        const rclcpp::Time stamp = Utils::getTime(time);
        try
        {
            ob = m_tf_buffer->lookupTransform(m_node_config.odom_frame, m_node_config.body_frame, stamp);
        }
        catch (const tf2::TransformException &)
        {
            // 정확한 시각이 버퍼에 없으면 가장 최근 값 사용
            try
            {
                ob = m_tf_buffer->lookupTransform(m_node_config.odom_frame, m_node_config.body_frame, tf2::TimePointZero);
            }
            catch (const tf2::TransformException &e)
            {
                RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "no %s -> %s TF (wheel odom): %s", m_node_config.odom_frame.c_str(),
                                     m_node_config.body_frame.c_str(), e.what());
                return;
            }
        }
        M3D r_wb;
        V3D t_wb;
        bodyPose(r_wb, t_wb);
        Eigen::Quaterniond q_ob(ob.transform.rotation.w, ob.transform.rotation.x, ob.transform.rotation.y, ob.transform.rotation.z);
        M3D r_ob = q_ob.normalized().toRotationMatrix();
        V3D t_ob(ob.transform.translation.x, ob.transform.translation.y, ob.transform.translation.z);
        M3D r_wo = r_wb * r_ob.transpose();
        V3D t_wo = t_wb - r_wo * t_ob;

        geometry_msgs::msg::TransformStamped ts;
        ts.header.frame_id = m_node_config.world_frame;
        ts.child_frame_id = m_node_config.odom_frame;
        ts.header.stamp = stamp;
        Eigen::Quaterniond q(r_wo);
        ts.transform.translation.x = t_wo.x();
        ts.transform.translation.y = t_wo.y();
        ts.transform.translation.z = t_wo.z();
        ts.transform.rotation.x = q.x();
        ts.transform.rotation.y = q.y();
        ts.transform.rotation.z = q.z();
        ts.transform.rotation.w = q.w();
        m_tf_broadcaster->sendTransform(ts);
    }

    void broadCastTF(std::shared_ptr<tf2_ros::TransformBroadcaster> broad_caster, std::string frame_id, std::string child_frame, const double &time)
    {
        M3D r_ob;
        V3D t_ob;
        bodyPose(r_ob, t_ob);
        geometry_msgs::msg::TransformStamped transformStamped;
        transformStamped.header.frame_id = frame_id;
        transformStamped.child_frame_id = child_frame;
        transformStamped.header.stamp = Utils::getTime(time);
        Eigen::Quaterniond q(r_ob);
        transformStamped.transform.translation.x = t_ob.x();
        transformStamped.transform.translation.y = t_ob.y();
        transformStamped.transform.translation.z = t_ob.z();
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broad_caster->sendTransform(transformStamped);
    }

    void timerCB()
    {
        if (!syncPackage())
            return;
        auto t1 = std::chrono::high_resolution_clock::now();
        m_builder->process(m_package);
        auto t2 = std::chrono::high_resolution_clock::now();

        if (m_node_config.print_time_cost)
        {
            auto time_used = std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
            RCLCPP_WARN(this->get_logger(), "Time cost: %.2f ms", time_used);
        }

        if (m_builder->status() != BuilderStatus::MAPPING)
            return;

        if (m_node_config.odom_frame.empty())
            broadCastTF(m_tf_broadcaster, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);
        else
            broadCastWorldToOdom(m_package.cloud_end_time);

        publishOdometry(m_odom_pub, m_node_config.world_frame, m_node_config.body_frame, m_package.cloud_end_time);

        // lidar -> body_frame: T_bi * T_il
        const M3D &r_bi = m_node_config.r_bi;
        const V3D &t_bi = m_node_config.t_bi;
        CloudType::Ptr body_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, r_bi * m_kf->x().r_il, r_bi * m_kf->x().t_il + t_bi);

        publishCloud(m_body_cloud_pub, body_cloud, m_node_config.body_frame, m_package.cloud_end_time);

        // lidar -> world_frame(odom): T_bi * T_wl
        CloudType::Ptr world_cloud = m_builder->lidar_processor()->transformCloud(m_package.cloud, r_bi * m_builder->lidar_processor()->r_wl(), r_bi * m_builder->lidar_processor()->t_wl() + t_bi);

        publishCloud(m_world_cloud_pub, world_cloud, m_node_config.world_frame, m_package.cloud_end_time);

        publishPath(m_path_pub, m_node_config.world_frame, m_package.cloud_end_time);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_lidar_sub;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr m_imu_sub;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_body_cloud_pub;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_world_cloud_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr m_path_pub;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr m_odom_pub;

    rclcpp::TimerBase::SharedPtr m_timer;
    StateData m_state_data;
    SyncPackage m_package;
    NodeConfig m_node_config;
    Config m_builder_config;
    std::shared_ptr<IESKF> m_kf;
    std::shared_ptr<MapBuilder> m_builder;
    std::shared_ptr<tf2_ros::TransformBroadcaster> m_tf_broadcaster;
    std::shared_ptr<tf2_ros::Buffer> m_tf_buffer;
    std::shared_ptr<tf2_ros::TransformListener> m_tf_listener;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LIONode>());
    rclcpp::shutdown();
    return 0;
}