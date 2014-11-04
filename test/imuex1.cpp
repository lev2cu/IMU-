

// void OdomInterface::publishPose()

void ImuFilter::publishPose(const ImuMsg::ConstPtr& imu_msg_raw)
{
  // **** create a copy of the pose and publish as shared pointer

  geometry_msgs::PoseStamped::Ptr pose_message = 
    boost::make_shared<geometry_msgs::PoseStamped>(pose_);

  pose_publisher_.publish(pose_message);

  // **** broadcast the transform

  tf::Stamped<tf::Pose> tf_pose;
  tf::poseStampedMsgToTF(pose_, tf_pose);
  tf::StampedTransform odom_to_base_link_tf(
    tf_pose, pose_.header.stamp, fixed_frame_, base_frame_);
  tf_broadcaster_.sendTransform(odom_to_base_link_tf);

  // *** publish odometry message

  nav_msgs::Odometry::Ptr odom_message = 
    boost::make_shared<nav_msgs::Odometry>();

  odom_message->header = pose_.header;
  odom_message->child_frame_id = "base_link";
  odom_message->pose.pose = pose_message->pose;
  
  odom_publisher_.publish(odom_message);
}







void ImuFilter::publishPose()
{
  
// create and publish fitlered Pose message
  boost::shared_ptr<ImuMsg> imu_pose = 
    boost::make_shared<ImuMsg>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);  
  imu_pose_.publish(pose);

}
