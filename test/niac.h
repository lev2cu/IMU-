#ifndef NIAC_H
#define NIAC_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <message_filters/subscriber.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
class NIAC
{

  public:
    NIAC(ros::NodeHandle nh, ros::NodeHandle nh_private);    
    virtual ~NIAC();

  private:
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> * sync_;
    message_filters::Subscriber<sensor_msgs::Imu> * imu_sub_;
    message_filters::Subscriber<geometry_msgs::Vector3Stamped> * lin_acc_sub_;
    //message_filters::TimeSynchronizer<sensor_msgs::Imu, geometry_msgs::Vector3Stamped> *sync_;
    // **** ROS-related
    ros::NodeHandle nh_;
    ros::NodeHandle nh_private_;
    ros::Subscriber pose_subscriber_;
    
    ros::Publisher pos_publisher_;    
    ros::Publisher acc_publisher_;
    ros::Publisher vel_publisher_;
    ros::Subscriber imu_subscriber_;
    ros::Publisher unf_pos_publisher_;  
    ros::Publisher unf_vel_publisher_;
    ros::Publisher unf_acc_publisher_;
    ros::Publisher lin_acc_diff_publisher_;
    ros::Publisher imu_delayed_publisher_;
    ros::Publisher imu_niac_publisher_;
    ros::Publisher imu_NIAC_publisher_;
    ros::Publisher roll_publisher_;
    ros::Publisher pitch_publisher_;
    ros::Publisher roll_kf_publisher_; 
    ros::Publisher pitch_kf_publisher_;
    ros::Publisher acc_mag_publisher_;
    ros::Publisher g_mag_publisher_;
    tf::TransformBroadcaster tf_broadcaster_;

    // **** paramaters
    double alpha_;
    double beta_;
    double threshold_;
    std::string fixed_frame_;
    std::string imu_frame_;
    std_msgs::Header imu_header_;
    double gain_;     // algorithm gain
    double gamma_;    // low pass filter gain
    tf::Transform f2b_prev_;
    tf::Vector3 imu_lin_acc_;
    tf::Vector3 lin_acc_diff_filtered_;
    tf::Vector3 lin_vel_;
    tf::Vector3 lin_acc_;
    geometry_msgs::Vector3 lin_acc_niac_;
    geometry_msgs::Vector3 lin_acc_vector_;
    std::vector<geometry_msgs::Vector3> ang_vel_;
    std::vector<geometry_msgs::Vector3> imu_acc_;
    std::vector<ros::Time> t_;

    tf::Vector3 pos_;
    geometry_msgs::Vector3 lin_acc_diff_vector_;
    geometry_msgs::PoseStamped::ConstPtr pose_msg_;
    // **** state variables
    ros::Time sync_time_;
    ros::Duration delay_;
    ros::Time timestamp_;
    bool initialized_;
    bool initialized_filter_;
    bool initialized_LP_filter_;
    ros::Time last_time_;
    ros::Time last_time_filter_;
    ros::Time last_time_imu_;
    boost::mutex mutex_;
    double latest_xpos_, latest_ypos_, latest_zpos_;
    double latest_xvel_, latest_yvel_, latest_zvel_;
    //std::vector<geometry_msgs::Vector3> ang_vel_;
    int imu_count_;
    bool vo_data_;

    double q0, q1, q2, q3;  // quaternion
    double constant_dt_;
    // **** member functions

    void initializeParams();
    void callback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw, const geometry_msgs::Vector3Stamped::ConstPtr lin_acc_msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void poseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg);
    void abgFilter(tf::Vector3 pos_reading, std_msgs::Header header, double dt);
    void madgwickAHRSupdateIMU(
      float gx, float gy, float gz,
      float ax, float ay, float az,
      float dt);
    void publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    void publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw);
    // Fast inverse square-root
        // See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
        static float invSqrt(float x)
        {
          float halfx = 0.5f * x;
          float y = x;
          long i = *(long*)&y;
          i = 0x5f3759df - (i>>1);
          y = *(float*)&i;
          y = y * (1.5f - (halfx * y * y));
          return y;
        }
};

#endif // NIAC_H
