void ImuFilter::imuPose(const ImuMsg::ConstPtr& imu_msg_raw, float dt)
{  
  const geometry_msgs::Vector3& lin_acc = imu_msg_raw->linear_acceleration; 
	geometry_msgs::Vector3 linW_acc;
  const geometry_msgs::Vector3& ang_vel = imu_msg_raw->angular_velocity;
    float anx, float any, float anz,
    
  // Detect stationary periods
  
  // Compute accelerometer magnitude
	if(!((lin_acc.x == 0.0f) && (lin_acc.y == 0.0f) && (lin_acc.z == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(lin_acc.x*lin_acc.x + lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z);
		anx = lin_acc.x*recipNorm;
		any = lin_acc.y*recipNorm;
		anz = lin_acc.z*recipNorm;
        
  double acc_mag = sqrt(lin_acc.x*lin_acc.x + lin_acc.y*lin_acc.y + lin_acc.z*lin_acc.z);


	//Compute gyroscope magnitude
  	if(!((ang_vel.x == 0.0f) && (ang_vel.y == 0.0f) && (ang_vel.z == 0.0f))) 
  {
		// Normalise accelerometer measurement
		recipNorm = invSqrt(ang_vel.x*ang_vel.x + ang_vel.y*ang_vel.y + ang_vel.z*ang_vel.z);
		gnx = ang_vel.x*recipNorm;
		gny = ang_vel.y*recipNorm;
		gnz = ang_vel.z*recipNorm;
        
	ROS_INFO("Gyros: %f %f %f", gnx, gny, gnz);
    }
	
  // force zero velocity when foot stationary

float thr = fabs(acc_mag - 9.81);

 if ( thr < g_Threshold_)
  {
    vel_.x = 0; 
    vel_.y = 0; 
    vel_.z = 0;
  }
  else 
  {

// Compute translational accelerations

/*
 // Rotate body accelerations to Earth frame	
tf::Quaternion q(q1, q2, q3, q0); 
tf::Quaternion qc(q1, q2, q3, q0);
tf::Quaternion ab(q1, q2, q3, q0); 
//quaternConj
    qc[0] = q0;
    qc[1] = -q1;
    qc[2] = -q2;
    qc[3] = -q3;

tf::Quaternion a_qc(0, lin_acc.x, lin_acc.y, lin_acc.z);   //ab << (q,v)
//ab = quaternProd(q, [zeros(row, 1) v])
    ab[0] = q[0]*a_qc[0]-q[1]*a_qc[1]-q[2]*a_qc[2]-q[3]*a_qc[3];
    ab[1] = q[0]*a_qc[1]+q[1]*a_qc[0]+q[2]*a_qc[3]-q[3]*a_qc[2];
    ab[2] = q[0]*a_qc[2]-q[1]*a_qc[3]+q[2]*a_qc[0]+q[3]*a_qc[1];
    ab[3] = q[0]*a_qc[3]+q[1]*a_qc[2]-q[2]*a_qc[1]+q[3]*a_qc[0];

//quaternProd(ab, quaternConj(q));
  //  ab[0] = ab[0]*qc[0]-ab[1]*qc[1]-ab[2]*qc[2]-ab[3]*qc[3];

    linW_acc.x = ab[0]*qc[1]+ab[1]*qc[0]+ab[2]*qc[3]-ab[3]*qc[2];
    linW_acc.y = ab[0]*qc[2]-ab[1]*qc[3]+ab[2]*qc[0]+ab[3]*qc[1];
    linW_acc.z = ab[0]*qc[3]+ab[1]*qc[2]-ab[2]*qc[1]+ab[3]*qc[0] - 9.81;
*/
 // Rotate body accelerations to Earth frame	
    linW_acc.x =  (q1*q1-q2*q2-q3*q3+q0*q0)*anx + 2*(q1*q2-q3*q0)*any +2*(q1*q3+q2*q0)*anz;                    
    linW_acc.y = 2*(q1*q2+q3*q0)*anx + (-q1*q1+q2*q2-q3*q3+q0*q0)*any +2*(q2*q3-q1*q0)*anz;
    linW_acc.z = (2*(q1*q3-q2*q0)*anx + 2*(q2*q3+q1*q0)*any + (-q1*q1-q2*q2+q3*q3+q0*q0)*anz)-9.81;

    // Compute translational velocities
    // Integrate acceleration to yield velocity
    vel_.x += linW_acc.x*dt;
    vel_.y += linW_acc.y*dt;
    vel_.z += linW_acc.z*dt;
    
    // Compute translational position
    // Integrate velocity to yield position
 /*   pos_.x += vel_.x*dt;
    pos_.y += vel_.y*dt;
    pos_.z += vel_.z*dt;
*/
    pos_.position.x = 0;
    pos_.position.y = 0;
    pos_.position.z = 0;
    pos_.orientation.x = q1;
    pos_.orientation.y = q2;
    pos_.orientation.z = q3;
    pos_.orientation.w = q0;
//	float [36] covariance = [];

	posCov_.pose = pos_;
//	posCov_.covariance = covariance;
	
	odom_.pose = posCov_;
	odom_.header = imu_msg_raw->header;
	odom_.header.frame_id = fixed_frame_;
	odom_.child_frame_id = imu_frame_;

 }

  i_odom_publisher_.publish(odom_);
  i_vel_publisher_.publish(vel_);
  i_acc_publisher_.publish(linW_acc);
  i_pos_publisher_.publish(pos_);

  //ROS_INFO("pos: %f %f %f", linW_acc.x, linW_acc.y, linW_acc.z);
}
}