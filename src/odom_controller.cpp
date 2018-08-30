#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <sensor_msgs/Imu.h>
#include <race/drive_values.h>
//==> Should define message type and make message package & add to dependency.
/* PWM LIMITS */

#define PI 3.14159265359
const double PWM_LOWER_LIMIT = 6554;
const double PWM_CENTER_VALUE = 9830;
const double PWM_UPPER_LIMIT = 13108;


/*
 * CAR SPECS
 * https://traxxas.com/products/models/electric/slash-4x4-tsm?t=specs
 * WHEELBASE: 324mm (distance between the front wheel and the rear wheel)
 * TRACK: 296mm
 * TIRE DIAMETER: 109.5mm
 * */
const double WHEELBASE = 0.324;  // 0.324 meter 



/*
 * Assume the car is using Ackermann Steering model.
 * 
 */

double curr_speed = 0.0; // Set this to linear.x speed
// **But what is "speed_to_erpm_offset_" thingy in mit-vesc_to_odom code????


double dt = 0.0; // last time - curr time
ros::Time curr_time(0.0);
ros::Time last_time(0.0);
double curr_angular_velocity = 0.0;
double curr_steering_angle = 0.0;

double yaw = 0.0;

// from IMU
double start_velocity = 0.0;
double linear_velocity_from_imu_x = 0.0;
double linear_velocity_from_imu_y = 0.0;
double angular_velocity_from_imu_z = 0.0;
double prev_linear_acceleration_imu_x = 0.0;
double prev_linear_acceleration_imu_y = 0.0;
double x_ = 0.0; // for pose estimation
double y_ = 0.0;




double mapValue (double toMap, double in_min, double in_max, double out_min, double out_max) {
    return (toMap - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void linearVelCallback(const std_msgs::Int32& vel_msg){
  // To be implemented
  //curr_time = ros::Time::now();

  // We're getting only linear x velocity here..
  curr_speed = (double) vel_msg.data;

  ROS_INFO("linearVelCallback");
  //dt = (curr_time - last_time).toSec();
  //last_time = curr_time;
}


void IMUCallback(const sensor_msgs::Imu& imuMsg){
  // Will take linear_acceleration.x & linear_acceleration.y to compare with our linear velocity
  double curr_linear_acceleration_imu_x = imuMsg.linear_acceleration.x;
  double curr_linear_acceleration_imu_y = imuMsg.linear_acceleration.y;

  double ux = (double) (curr_linear_acceleration_imu_x + prev_linear_acceleration_imu_x)/2 * dt;
  double uy = (double) (curr_linear_acceleration_imu_y + prev_linear_acceleration_imu_y)/2 * dt;
  
   
  // s = v_0*t + 1/2 * a * t^2 (this is distance traveled.. isn't it?)
  // v_0 = ?
  //linear_velocity_from_imu_x += ux; // Is this right?
  linear_velocity_from_imu_y += uy; // Is this right?
  linear_velocity_from_imu_x = start_velocity * dt + 1/2 * curr_linear_acceleration_imu_x * dt * dt;

  // What exactly should go into pose.position.x/y??? Is it velocity or distance??
  angular_velocity_from_imu_z = imuMsg.angular_velocity.z; 
  // will give you a velocity drift

  prev_linear_acceleration_imu_x = curr_linear_acceleration_imu_x;
  prev_linear_acceleration_imu_y = curr_linear_acceleration_imu_y;
  // should I assign separate prev time variable for imu?
  
  ROS_INFO("IMUCallback");
}


void driveCallback(const race::drive_values& drive_value){
  // msg type 'drive_value': ranges from 6554 to 13108 (center value: 9830) // f1-tenth set
  // Get steering angle from the incoming message
  // 6554: 0 degree, 9830: 90 degree, 13108: 180 degree (????????)
  double mapped_angle = mapValue(drive_value.pwm_angle, PWM_LOWER_LIMIT, PWM_UPPER_LIMIT, 90, -90);//left -> pwm gets higher
  
  // curr_steering_angle: radians
  curr_steering_angle = (double) (mapped_angle * PI) / 180;
}




int main(int argc, char** argv){
  
  ros::init(argc, argv, "fuckingodom");
  ros::NodeHandle n;
  
  
  ros::Subscriber vel_sub = n.subscribe("/linear_vel", 50, linearVelCallback);
  ros::Subscriber steer_sub = n.subscribe("/drive_pwm", 50, driveCallback);
  ros::Subscriber imu_linear = n.subscribe("/imu", 50, IMUCallback);
  
  ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 50);
  ros::Publisher testpub = n.advertise<std_msgs::String>("test", 50);
  

  tf::TransformBroadcaster odom_broadcaster;

  ros::Rate r(10.0);

  int odom_mode = 1; // 0: MIT, 1: linorobot
  
  while(n.ok()){
    ros::spinOnce();
    curr_time = ros::Time::now();
    
    nav_msgs::Odometry odomMsg;
    odomMsg.header.stamp = curr_time;
    odomMsg.header.frame_id = "odom";
    odomMsg.child_frame_id = "base_link"; //  Or "base_footprint"?
    geometry_msgs::TransformStamped odomTf;


    /* 6/20 memo 
     *
     * 1. TF not resolved yet!!
     * 2. What's position?
     * 3. Steering angle range in degrees not clear yet.
     * 4. Not committed oscar_drive.ino yet.
     * */


    // <m Message Twist-Twist-Linear/Angular>
    // MIT-like current angular velocity
    // curr_steering_angle => Is this accurate??? *****
    curr_angular_velocity = (double) curr_speed * tan(curr_steering_angle) / WHEELBASE;
    yaw += (curr_angular_velocity * dt); 
 

    // <Odom Message Pose-Position>
    // "position"?
    double x_dot = curr_speed * cos(yaw);
    double y_dot = curr_speed * sin(yaw);
    x_ += x_dot * dt;
    y_ += y_dot * dt;
    odomMsg.pose.pose.position.x = x_;
    odomMsg.pose.pose.position.y = y_;

    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    odomMsg.pose.pose.orientation = odom_quat;
    /*
    odomMsg.pose.pose.orientation.x = 0.0;
    odomMsg.pose.pose.orientation.y = 0.0;
    odomMsg.pose.pose.orientation.z = sin(yaw/2.0);// ?
    odomMsg.pose.pose.orientation.w = cos(yaw/2.0);// ?
    */
    //=>Or should we use tf::createQuaternionMsgFromYaw(theta) instead?
    
    odomTf.header.stamp = ros::Time::now();
    odomTf.header.frame_id = "odom";
    odomTf.child_frame_id = "base_link"; //  Or "base_footprint"?
    odomTf.transform.translation.x = x_;
    odomTf.transform.translation.y = y_;
    odomTf.transform.translation.z = 0.0;
    odomTf.transform.rotation = odomMsg.pose.pose.orientation;
    odom_broadcaster.sendTransform(odomTf);

   
    if(odom_mode == 0){
      // *********************************************** MIT-style odom message
      odomMsg.twist.twist.linear.x = curr_speed;
      odomMsg.twist.twist.linear.y = 0.0; // **Not zero in linorobot's code
      //odomMsg.twist.twist.linear.z = 0.0;
      //odomMsg.twist.twist.angular.x = 0.0;
      //odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = curr_angular_velocity; 
      // *********************************************** MIT-style odom message
    }
    else{//odom_mode: 1
      // linorobot uses linear velocity x, y from wheel encoder.
      // and angular velocity z from imu.
      // But here I'm going to compare imu-data-generated linear velocity with hall-sensor-rpm-generated linear velocity.
      //
      // *********************************************** linorobot-style odom message
      odomMsg.twist.twist.linear.x = linear_velocity_from_imu_x;
      odomMsg.twist.twist.linear.y = linear_velocity_from_imu_y;
      odomMsg.twist.twist.linear.z = 0.0;

      odomMsg.twist.twist.angular.x = 0.0;
      odomMsg.twist.twist.angular.y = 0.0;
      odomMsg.twist.twist.angular.z = angular_velocity_from_imu_z; 
      // *********************************************** linorobot-style odom message
    }

    //==> **Angular speed in from IMU in linorobot's code / derived from current steering angle in mit's code.
    //linorobot uses wheel encoder or external libraries.. no idea how it gets linear y velocity.
    
    odom_pub.publish(odomMsg);
    
    std_msgs::String testmsg;
    std::stringstream ss;
    ss << linear_velocity_from_imu_x;
    testmsg.data = ss.str();
    testpub.publish(testmsg);

    
    dt = (curr_time - last_time).toSec();
    last_time = curr_time;

    r.sleep();
  }

}
