
#include "NIAC/niac.h"

NIAC::NIAC(ros::NodeHandle nh, ros::NodeHandle nh_private):
  nh_(nh), 
  nh_private_(nh_private),
  initialized_(false),
  initialized_filter_(false),
  initialized_LP_filter_(false),
  q0(1.0), q1(0.0), q2(0.0), q3(0.0)
{
  ROS_INFO ("Starting NIAC");

  // **** get paramters 
  // nothing for now

  initializeParams();
  
  int queue = 100;

  // **** register publishers
  unf_pos_publisher_ = nh_.advertise<geometry_msgs::Vector3>(
    "unfilt_pos", queue);
  unf_vel_publisher_ = nh_.advertise<geometry_msgs::Vector3>(
    "unf_lin_vel", queue);
  unf_acc_publisher_ = nh_.advertise<geometry_msgs::Vector3>(
    "unf_lin_acc", queue);

  pos_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
    "filt_pos", queue);
  vel_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
    "lin_vel", queue);
  acc_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
    "lin_acc", queue);
  lin_acc_diff_publisher_ = nh_.advertise<geometry_msgs::Vector3Stamped>(
		    "lin_acc_diff", queue);

  imu_delayed_publisher_ = nh_.advertise<sensor_msgs::Imu>(
		  "imu_delayed/data_raw", queue);//"imu_delayed", queue);
  imu_niac_publisher_ = nh_.advertise<sensor_msgs::Imu>(
    		    "imu_niac", queue);
  imu_NIAC_publisher_ = nh_.advertise<sensor_msgs::Imu>(
	    "imu_NIAC/data", queue);

  roll_publisher_    = nh.advertise<std_msgs::Float32>("roll_niac", queue);
  pitch_publisher_   = nh.advertise<std_msgs::Float32>("pitch_niac", queue);
  roll_kf_publisher_    = nh.advertise<std_msgs::Float32>("roll_kf", queue);
  pitch_kf_publisher_   = nh.advertise<std_msgs::Float32>("pitch_kf", queue);
  acc_mag_publisher_ = nh.advertise<std_msgs::Float32>("acc_mag", queue);
  g_mag_publisher_   = nh.advertise<std_msgs::Float32>("g_mag", queue);

  // **** register subscribers
  int queue_size = 100;

  imu_subscriber_ = nh_.subscribe(
    "imu/data_raw", queue_size, &NIAC::imuCallback, this);//"imu/data_raw", queue_size, &NIAC::imuCallback, this);

  pose_subscriber_ = nh_.subscribe(
     "pose", 10, &NIAC::poseCallback, this);

  imu_sub_  = new message_filters::Subscriber<sensor_msgs::Imu>(nh_, "imu_delayed/data_raw", queue_size); //(nh_, "/imu_delayed", queue_size);
  lin_acc_sub_ = new message_filters::Subscriber<geometry_msgs::Vector3Stamped>(nh_, "lin_acc", queue_size);
  sync_ = new message_filters::Synchronizer<MySyncPolicy>(MySyncPolicy(queue_size), *imu_sub_, *lin_acc_sub_);
  sync_->registerCallback(boost::bind(&NIAC::callback, this, _1, _2));
}

NIAC::~NIAC()
{
  ROS_INFO ("Destroying NIAC");
}

void NIAC::initializeParams()
{
  f2b_prev_.setIdentity();
  latest_xpos_ = 0.0;
  latest_ypos_ = 0.0;
  latest_zpos_ = 0.0;
  latest_xvel_ = 0.0;
  latest_yvel_ = 0.0;
  latest_zvel_ = 0.0;
  imu_count_   = 0;
  vo_data_     = false;
  //last_time_ = 0.0;
 // initialized_ = false;
  if (!nh_private_.getParam ("alpha", alpha_))
    alpha_ = 0.9;
  if (!nh_private_.getParam ("beta", beta_))
    beta_ = 0.5;
  if (!nh_private_.getParam ("gamma", gamma_))
    gamma_ = 0.2;
  if (!nh_private_.getParam ("constant_dt", constant_dt_))
      constant_dt_ = 0.0;
  if (!nh_private_.getParam ("gain", gain_))
     gain_ = 0.1;
  if (!nh_private_.getParam ("fixed_frame", fixed_frame_))
     fixed_frame_ = "odom";
  if (!nh_private_.getParam ("imu_frame", imu_frame_))
       imu_frame_ = "imu_niac";
  if (!nh_private_.getParam ("threshold", threshold_))
       threshold_ = 10.0;
}



void NIAC::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{

  boost::mutex::scoped_lock(mutex_);

  sensor_msgs::Imu imu =  *imu_msg_raw;
  
  imu.header.stamp = ros::Time::now() + delay_;// + ros::Duration(0.1);//;
  imu_delayed_publisher_.publish(imu);
  
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 

  ros::Time time = imu_msg_raw->header.stamp;
  

  float a_x = lin_acc.x;
  float a_y = lin_acc.y;
  float a_z = lin_acc.z;

  float a_magnitude = sqrt(a_x*a_x + a_y*a_y + a_z*a_z);
  
  if (!initialized_filter_)
    {
      
      // initialize roll/pitch orientation from acc. vector
  	  double roll  = atan2(a_y, sqrt(a_x*a_x + a_z*a_z));
  	  double pitch = atan2(-a_x, sqrt(a_y*a_y + a_z*a_z));
  	  double yaw = 0.0;

      tf::Quaternion init_q = tf::createQuaternionFromRPY(roll, pitch, yaw);

      q1 = init_q.getX();
      q2 = init_q.getY();
      q3 = init_q.getZ();
      q0 = init_q.getW();

      // initialize time
      last_time_filter_ = time;
      initialized_filter_ = true;
    }
    // determine dt: either constant, or from IMU timestamp
    float dt;
    if (constant_dt_ > 0.0)
  	  dt = constant_dt_;
    else 
    	dt = (time - last_time_filter_).toSec();
   
    last_time_filter_ = time;
    

       /*
  if (a_magnitude <= threshold_)
  {
	  madgwickAHRSupdateIMU(
	      	        ang_vel.x, ang_vel.y, ang_vel.z,
	      	        lin_acc.x, lin_acc.y, lin_acc.z,
	      	    	  dt);

  }*/

  if((a_magnitude > threshold_) || (a_magnitude < -threshold_))
  //if (a_magnitude > threshold_)  
  {
    
	  if (!vo_data_)
	  	    {
		  	  madgwickAHRSupdateIMU(
		  		  	  	        ang_vel.x, ang_vel.y, ang_vel.z,
		  		  	  	        0,0,0,
		  		  	  	    	  dt);
	  	    }
	  else
	  	  {
	  madgwickAHRSupdateIMU(
			  ang_vel.x, ang_vel.y, ang_vel.z,
			  lin_acc_diff_vector_.x, lin_acc_diff_vector_.y, lin_acc_diff_vector_.z,
	  		  dt);
	  	  

         }

    }
  else
  {
	  madgwickAHRSupdateIMU(
	      	        ang_vel.x, ang_vel.y, ang_vel.z,
	      	        lin_acc.x, lin_acc.y, lin_acc.z,
	      	    	  dt);
  }
/*
    std_msgs::Float32 acc_mag_msg;
    acc_mag_msg.data = a_magnitude;
    acc_mag_publisher_.publish(acc_mag_msg);*/
    publishFilteredMsg(imu_msg_raw);
    publishTransform(imu_msg_raw);
   
}

void NIAC::publishTransform(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  double r_raw ,p_raw ,y_raw;
  tf::Quaternion q_raw;
  tf::quaternionMsgToTF(imu_msg_raw->orientation, q_raw);
  tf::Matrix3x3 M;
  M.setRotation(q_raw);
  M.getRPY(r_raw, p_raw, y_raw);

  double r, p, y;
  tf::Quaternion q(q1, q2, q3, q0);
  M.setRotation(q);
  M.getRPY(r, p, y);

  tf::Quaternion q_final = tf::createQuaternionFromRPY(r, p, y_raw);
  //tf::Quaternion q(q1, q2, q3, q0);
  tf::Transform transform;
  transform.setOrigin( tf::Vector3( 0.0, 0.0, 0.0 ) );
  transform.setRotation( q_final );
  tf_broadcaster_.sendTransform( tf::StampedTransform( transform,
                   imu_msg_raw->header.stamp,
                   fixed_frame_,
                   imu_frame_ ) );

}

void NIAC::publishFilteredMsg(const sensor_msgs::Imu::ConstPtr& imu_msg_raw)
{
  // create orientation quaternion
  // q0 is the angle, q1, q2, q3 are the axes
  tf::Quaternion q(q1, q2, q3, q0);

  // create and publish fitlered IMU message
  boost::shared_ptr<sensor_msgs::Imu> imu_msg = boost::make_shared<sensor_msgs::Imu>(*imu_msg_raw);

  imu_msg->header.frame_id = fixed_frame_;
  tf::quaternionTFToMsg(q, imu_msg->orientation);
  imu_NIAC_publisher_.publish(imu_msg);

  double roll, pitch, y;
  tf::Matrix3x3 M;
  M.setRotation(q);
  M.getRPY(roll, pitch, y);
  std_msgs::Float32 roll_msg;
  std_msgs::Float32 pitch_msg;
  roll_msg.data = roll;
  pitch_msg.data = pitch;
  roll_publisher_.publish(roll_msg);
  pitch_publisher_.publish(pitch_msg);
  
   /********* XSens roll and pitch publishing ************/
   tf::Quaternion q_kf; 
   tf::quaternionMsgToTF(imu_msg_raw->orientation, q_kf);
   M.setRotation(q_kf);
   M.getRPY(roll, pitch, y);
   roll_msg.data = roll;
   pitch_msg.data = pitch;
   roll_kf_publisher_.publish(roll_msg);
   pitch_kf_publisher_.publish(pitch_msg);
}

void NIAC::madgwickAHRSupdateIMU(
  float gx, float gy, float gz,
  float ax, float ay, float az,
  float dt)
{
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
	qDot2 = 0.5f * ( q0 * gx + q2 * gz - q3 * gy);
	qDot3 = 0.5f * ( q0 * gy - q1 * gz + q3 * gx);
	qDot4 = 0.5f * ( q0 * gz + q1 * gy - q2 * gx);

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f)))
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ax * ax + ay * ay + az * az);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q0;
		_2q1 = 2.0f * q1;
		_2q2 = 2.0f * q2;
		_2q3 = 2.0f * q3;
		_4q0 = 4.0f * q0;
		_4q1 = 4.0f * q1;
		_4q2 = 4.0f * q2;
		_8q1 = 8.0f * q1;
		_8q2 = 8.0f * q2;
		q0q0 = q0 * q0;
		q1q1 = q1 * q1;
		q2q2 = q2 * q2;
		q3q3 = q3 * q3;

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= gain_ * s0;
		qDot2 -= gain_ * s1;
		qDot3 -= gain_ * s2;
		qDot4 -= gain_ * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q0 += qDot1 * dt;
	q1 += qDot2 * dt;
	q2 += qDot3 * dt;
	q3 += qDot4 * dt;

	// Normalise quaternion
	recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 *= recipNorm;
	q1 *= recipNorm;
	q2 *= recipNorm;
	q3 *= recipNorm;
}


void NIAC::poseCallback(const geometry_msgs::PoseStamped::ConstPtr pose_msg)
{
  //vo_data_ = true;
  ros::Duration time_diff = ros::Time::now() - last_time_filter_;
  //ROS_INFO("TIME_DIFF in ms: %f", time_diff.nsec/1e6);
  tf::Stamped<tf::Transform> f2b_stamped;
  tf::poseStampedMsgToTF(*pose_msg, f2b_stamped);
  tf::Transform f2b = f2b_stamped;
  std_msgs::Header header = pose_msg->header;
  tf::Transform delta_pose = f2b_prev_.inverse() * f2b;
  f2b_prev_ = f2b;

  tf::Vector3 delta_position = delta_pose.getOrigin();

  ros::Time time = pose_msg->header.stamp;  
  double dt = (time - last_time_).toSec();
  
  geometry_msgs::Vector3Stamped pos;
  pos.vector.x = pose_msg->pose.position.x;
  pos.vector.y = pose_msg->pose.position.y;
  pos.vector.z = pose_msg->pose.position.z;

  pos.header = pose_msg->header;

  last_time_ = time;
  
  geometry_msgs::Vector3 position;
  position.x = pose_msg->pose.position.x;
  position.y = pose_msg->pose.position.y;
  position.z = pose_msg->pose.position.z;
  
  geometry_msgs::Vector3 vel;
  double vel_x = (pose_msg->pose.position.x - latest_xpos_) / dt;
  double vel_y = (pose_msg->pose.position.y - latest_ypos_) / dt;
  double vel_z = (pose_msg->pose.position.z - latest_zpos_) / dt;
  latest_xpos_ = pose_msg->pose.position.x; 
  latest_ypos_ = pose_msg->pose.position.y;
  latest_zpos_ = pose_msg->pose.position.z;
  
  geometry_msgs::Vector3 lin_acc;
  lin_acc.x = (vel_x - latest_xvel_) / dt;
  lin_acc.y = (vel_y - latest_yvel_) / dt;
  lin_acc.z = (vel_z - latest_zvel_) / dt;
  latest_xvel_ = vel_x; 
  latest_yvel_ = vel_y;
  latest_zvel_ = vel_z;
  vel.x = vel_x; vel.y=vel_y; vel.z=vel_z;
  unf_pos_publisher_.publish(position);
  unf_vel_publisher_.publish(vel);
  unf_acc_publisher_.publish(lin_acc);
  delay_ = ros::Time::now()-header.stamp;
  double delay_ns = delay_.nsec/1e9;
  //ROS_INFO("NIAC vs RGBD delay in sec: %f \n", delay_ns );

  abgFilter(delta_position, header, dt);
}

void NIAC::callback(const sensor_msgs::Imu::ConstPtr& imu_msg_raw, const geometry_msgs::Vector3Stamped::ConstPtr lin_acc_msg)
{

	boost::mutex::scoped_lock(mutex_);

  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
  lin_acc_niac_ = lin_acc_msg->vector;
  timestamp_ = lin_acc_msg->header.stamp;
  vo_data_ = true;

  tf::Vector3 lin_acc_niac;
  tf::vector3MsgToTF(imu_msg_raw->linear_acceleration, imu_lin_acc_);
  //imu_frame_ = imu_msg_raw->header.frame_id;
  //tf::vector3MsgToTF(lin_acc_msg, lin_acc_niac_);
  tf::vector3MsgToTF(lin_acc_niac_, lin_acc_niac );


  tf::Vector3 lin_acc_diff = imu_lin_acc_ - lin_acc_niac;
  ROS_INFO("DIFF DONE");
  if(!initialized_LP_filter_)
  {
    initialized_LP_filter_ = true;
    lin_acc_diff_filtered_ = lin_acc_diff; 
  }
  else
  {
    //low pass filter to remove the noise in the linear acceleration difference
    lin_acc_diff_filtered_ += gamma_ * (lin_acc_diff - lin_acc_diff_filtered_);
  }
  
geometry_msgs::Vector3 lin_acc_diff_filtered_vector;
  tf::vector3TFToMsg(lin_acc_diff_filtered_, lin_acc_diff_filtered_vector);
  double g_x_imu = imu_msg_raw->linear_acceleration.x;
  double g_y_imu = imu_msg_raw->linear_acceleration.y;
  double g_z_imu = imu_msg_raw->linear_acceleration.z;
  double g_x = lin_acc_diff_filtered_vector.x;
  double g_y = lin_acc_diff_filtered_vector.y;
  double g_z = lin_acc_diff_filtered_vector.z; 

  double g_imu_magnitude = sqrt(g_x_imu * g_x_imu + g_y_imu * g_y_imu + g_z_imu * g_z_imu);  
  double g_niac_magnitude = sqrt(g_x * g_x + g_y * g_y + g_z * g_z);
  double g_magnitude;

  if(abs(g_imu_magnitude - 9.81) > threshold_)
    g_magnitude = g_niac_magnitude;
  else
    g_magnitude = g_imu_magnitude;

  std_msgs::Float32 g_mag_msg;
  g_mag_msg.data = g_magnitude;
  g_mag_publisher_.publish(g_mag_msg);

  std_msgs::Float32 acc_mag_msg;
  acc_mag_msg.data = g_imu_magnitude;
  acc_mag_publisher_.publish(acc_mag_msg);

  //geometry_msgs::Vector3 lin_acc_diff_vector_;
  //tf::vector3TFToMsg(lin_acc_diff, lin_acc_diff_vector_);
  tf::vector3TFToMsg(lin_acc_diff_filtered_, lin_acc_diff_vector_);
  geometry_msgs::Vector3Stamped lin_acc_diff_msg;
  lin_acc_diff_msg.header = imu_msg_raw->header;
  lin_acc_diff_msg.vector = lin_acc_diff_vector_;
  lin_acc_diff_publisher_.publish(lin_acc_diff_msg);

  //float ax = imu_msg_raw->linear_acceleration.x;
  //float ay = imu_msg_raw->linear_acceleration.y;
  //float az = imu_msg_raw->linear_acceleration.z;

  sensor_msgs::Imu imu_niac = *imu_msg_raw;

  //double g_mag = sqrt(ax*ax + ay*ay + az*az);

  //ROS_INFO("acceleration magnitude: %f \n", g_mag);

 //if (g_mag > threshold_)
  imu_niac.linear_acceleration = lin_acc_diff_vector_; //lin_acc_niac_;

  imu_niac_publisher_.publish(imu_niac);
  ros::Time time = imu_msg_raw->header.stamp;

 }




void NIAC::abgFilter(tf::Vector3 delta_pos_reading, std_msgs::Header header, double dt)
{
  
  tf::Vector3 lin_vel_reading = delta_pos_reading/dt;
  
  if(!initialized_)
  {
    initialized_ = true;
  
    lin_vel_ = lin_vel_reading;
    
    // set initial acceleration to 0
    lin_acc_ = tf::Vector3(0.0, 0.0, 0.0); 
  }
  else
  {
	
	  double bdt = beta_ / dt;
	  tf::Vector3 lin_vel_pred = lin_vel_ + dt * lin_acc_;
	  tf::Vector3 r_vel = lin_vel_reading - lin_vel_pred;
	  lin_vel_ = lin_vel_pred + alpha_ * r_vel;
	  lin_acc_ = lin_acc_ + bdt * r_vel;
    }
    // publish
	
    geometry_msgs::Vector3Stamped lin_vel_msg;
    geometry_msgs::Vector3 lin_vel_vector;
    tf::vector3TFToMsg(lin_vel_, lin_vel_vector);
    lin_vel_msg.vector = lin_vel_vector;
    lin_vel_msg.header = header;
    vel_publisher_.publish(lin_vel_msg);
    
    geometry_msgs::Vector3Stamped lin_acc_msg;
    geometry_msgs::Vector3 lin_acc_vector;
    tf::vector3TFToMsg(lin_acc_, lin_acc_vector);
    lin_acc_msg.vector = lin_acc_vector;
    lin_acc_msg.header.frame_id = header.frame_id;
    lin_acc_msg.header.stamp = header.stamp;//imu_header_;ros::Time::now();
    //lin_acc_msg.header = header;
    acc_publisher_.publish(lin_acc_msg);
    /*
    tf::Vector3 lin_acc_diff = imu_lin_acc_ - lin_acc_;
    geometry_msgs::Vector3 lin_acc_diff_msg;
    tf::vector3TFToMsg(lin_acc_diff, lin_acc_diff_msg);
    lin_acc_diff_publisher_.publish(lin_acc_diff_msg);

    */
  
}



  

