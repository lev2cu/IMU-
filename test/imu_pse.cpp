
void ImuFilter::imuPose(const ImuMsg::ConstPtr& imu_msg_raw, dt)
{
const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
tf::Vector3 lin_vel;
tf::Vector3 linW_acc;
tf::Vector3 linW_pos;
// Detect stationary periods

// Compute accelerometer magnitude
// acc_mag = sqrt(accX.*accX + accY.*accY + accZ.*accZ);

double acc_mag = sqrt(lin_acc.x*lin_acc.x + lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z);

// Threshold detection
if ( fabs(acc_mag - 9.81) < 0.1
{
  lin_vel = tf::Vector3(0.0, 0.0, 0.0);
}
else 
{
//     % Compute translational accelerations
// % Rotate body accelerations to Earth frame
            
    linW_acc.x =  (q1_*q1_-q2_*q2_-q3_*q3_+q4_*q4_)*lin_acc.x + 2*(q1_*q2_+q3_*q4_)*lin_acc.y + 2*(q1_*q3_-q2_*q4_)*lin_acc.z;
    linW_acc.y = 2*(q1_*q2_-q3_*q4_)*lin_acc.x + (-q1_*q1_+q2_*q2_-q3_*q3_+q4_*q4_)*lin_acc.y + 2*(q2_*q3_+q1_*q4_)*lin_acc.z;
    linW_acc.z = 2*(q1_*q3_+q2_*q4_)*lin_acc.x + 2*(q2_*q3_-q1_*q4_)*lin_acc.y + (-q1_*q1_-q2_*q2_+q3_*q3_+q4_*q4_)*lin_acc.z - 9.81;
}
    lin_vel = linP_vel_ + linW_acc*dt;
    linP_vel_ = lin_vel;
    
    linW_pos = linP_pos_ + lin_vel*dt;
    linP_pos = linW_pos;
}