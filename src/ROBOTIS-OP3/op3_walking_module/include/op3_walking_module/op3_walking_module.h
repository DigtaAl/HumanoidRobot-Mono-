/*******************************************************************************
* Copyright 2017 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Author: Kayman */

#ifndef OP3_WALKING_MODULE_H_
#define OP3_WALKING_MODULE_H_

#include "op3_walking_parameter.h"

#include <stdio.h>
#include <math.h>
#include <fstream>
#include <boost/thread.hpp>
#include <eigen3/Eigen/Eigen>
#include <yaml-cpp/yaml.h>
#include <chrono>
#include <algorithm>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>
#include <eigen_conversions/eigen_msg.h>

#include "robotis_controller_msgs/StatusMsg.h"
#include "robotis_controller_msgs/SensorYPR.h"
#include "op3_walking_module_msgs/WalkingParam.h"
#include "op3_walking_module_msgs/GetWalkingParam.h"
#include "op3_walking_module_msgs/SetWalkingParam.h"
#include "op3_walking_module_msgs/WalkingCorrection.h"

#include "robotis_framework_common/motion_module.h"
#include "robotis_math/robotis_math.h"
#include "robotis_math/robotis_math_base.h"
#include "robotis_math/robotis_trajectory_calculator.h"
#include "op3_kinematics_dynamics/op3_kinematics_dynamics.h"

namespace robotis_op
{

  # define M_PIWalk 3.14159265358979323846


// ===============================    PID 
class WalkPID{
  public:

  double sampleTime;
  double lastTime;
  double lastInput;
  double target;
  double output;
  double errorSum;

  double p;
  double i;
  double d;

  double min = -1;
  double max = 1;

  double applied = 0;

  WalkPID(){};
  WalkPID (double _p, double _i, double _d){
    // std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
    //     std::chrono::system_clock::now().time_since_epoch()
    // );

    sampleTime = 500;
    target = 0;
    output = 0;
    errorSum = 0;
    lastInput = 0;
    // lastTime = ms.count() - sampleTime;

    setTunings(_p, _i, _d);
  };
  
  void setTunings(double _p, double _i, double _d){
    double ratio = sampleTime / 1000;
    p = _p;
    i = _i * ratio;
    d = _d / ratio;
  };

  void setSampleTime(double _sampleTime){
    double ratio = sampleTime / _sampleTime;
    i *= ratio;
    d /= ratio;
    sampleTime = _sampleTime;
  };

  void setOutputLimits(double _min, double _max){
    min = _min;
    max = _max;
  };

  void setTarget(double _target){
    target = _target;
  };

  void apply(){
    applied = output;
  };

  double compute(double input){
    // std::chrono::milliseconds now = std::chrono::duration_cast< std::chrono::milliseconds >(
    //     std::chrono::system_clock::now().time_since_epoch()
    // );

    // double time_diff = now.count() - lastTime;

    // if(time_diff >= sampleTime){
      double error = target - input;
      double inputDiff = input - lastInput;

      errorSum = std::max(min, std::min(max, errorSum + (i * error)));
      output = std::max(min, std::min(max, (p * error) + errorSum - (d * inputDiff)));
      lastInput = input;
      // lastTime = now.count();
    // }

    return output;
  }
};



typedef struct
{
  double x, y, z;
} Position3D;

typedef struct
{
  double x, y, z, roll, pitch, yaw;
} Pose3D;

class WalkingModule : public robotis_framework::MotionModule, public robotis_framework::Singleton<WalkingModule>
{

 public:
  enum
  {
    PHASE0 = 0,
    PHASE1 = 1,
    PHASE2 = 2,
    PHASE3 = 3
  };

  WalkingModule();
  virtual ~WalkingModule();

  void initialize(const int control_cycle_msec, robotis_framework::Robot *robot);
  void process(std::map<std::string, robotis_framework::Dynamixel *> dxls, std::map<std::string, double> sensors);
  void stop();
  bool isRunning();
  void onModuleEnable();
  void onModuleDisable();

  int getCurrentPhase()
  {
    return phase_;
  }
  double getBodySwingY()
  {
    return body_swing_y;
  }
  double getBodySwingZ()
  {
    return body_swing_z;
  }

 private:
  enum
  {
    WalkingDisable = 0,
    WalkingEnable = 1,
    WalkingInitPose = 2,
    WalkingReady = 3
  };

  const bool DEBUG;

  void queueThread();

  /* ROS Topic Callback Functions */
  void walkingCommandCallback(const std_msgs::String::ConstPtr &msg);
  void walkingParameterCallback(const op3_walking_module_msgs::WalkingParam::ConstPtr &msg);
  bool getWalkigParameterCallback(op3_walking_module_msgs::GetWalkingParam::Request &req,
                                  op3_walking_module_msgs::GetWalkingParam::Response &res);
  void angleCorrection(const robotis_controller_msgs::SensorYPR::ConstPtr &ypr);
  void configChangedCallback(const op3_walking_module_msgs::WalkingCorrection &walkCorrection);

  /* ROS Service Callback Functions */
  void processPhase(const double &time_unit);
  bool computeLegAngle(double *leg_angle);
  void computeArmAngle(double *arm_angle);
  void sensoryFeedback(const double &rlGyroErr, const double &fbGyroErr, double *balance_angle);

  void publishStatusMsg(unsigned int type, std::string msg);
  double wSin(double time, double period, double period_shift, double mag, double mag_shift);
  bool computeIK(double *out, double x, double y, double z, double a, double b, double c);
  void updateTimeParam();
  void updateMovementParam();
  void updatePoseParam();
  void startWalking();
  void loadWalkingParam(const std::string &path);
  void saveWalkingParam(std::string &path);
  void iniPoseTraGene(double mov_time);
  void setZeroAngle();
  double pidWalkXcorrection();
  void sendMonitorCorrection(unsigned long deltaT, double inputPitch, double correction);

  double zeroPitch = 0;
  double zeroOffsetScale = 10;
  double zeroKickStartScale = 10;
  double zeroPitchOffset = 0;
  double PIDWalkScale = 15;
  double sensorPitch = 0;
  double xOffsetMultiplier = 0.006;
  WalkPID pitchPID;

  unsigned long long startTime = 0;
  unsigned long long lastTimePID = 0;
  unsigned long intervalPID = 100;
  double lastPIDResult = 0;

  OP3KinematicsDynamics* op3_kd_;
  int control_cycle_msec_;
  std::string param_path_;
  boost::thread queue_thread_;
  boost::mutex publish_mutex_;

  /* ROS Topic Publish Functions */
  ros::Publisher robot_pose_pub_;
  ros::Publisher status_msg_pub_;
  ros::Publisher balance_monitor_pub_;

  Eigen::MatrixXd calc_joint_tra_;

  Eigen::MatrixXd target_position_;
  Eigen::MatrixXd goal_position_;
  Eigen::MatrixXd init_position_;
  Eigen::MatrixXi joint_axis_direction_;
  std::map<std::string, int> joint_table_;
  int walking_state_;
  int init_pose_count_;
  op3_walking_module_msgs::WalkingParam walking_param_;
  double previous_x_move_amplitude_;

  // variable for walking
  double period_time_;
  double dsp_ratio_;
  double ssp_ratio_;
  double x_swap_period_time_;
  double x_move_period_time_;
  double y_swap_period_time_;
  double y_move_period_time_;
  double z_swap_period_time_;
  double z_move_period_time_;
  double a_move_period_time_;
  double ssp_time_;
  double l_ssp_start_time_;
  double l_ssp_end_time_;
  double r_ssp_start_time_;
  double r_ssp_end_time_;
  double phase1_time_;
  double phase2_time_;
  double phase3_time_;

  double x_offset_;
  double y_offset_;
  double z_offset_;
  double r_offset_;
  double p_offset_;
  double a_offset_;

  double x_swap_phase_shift_;
  double x_swap_amplitude_;
  double x_swap_amplitude_shift_;
  double x_move_phase_shift_;
  double x_move_amplitude_;
  double x_move_amplitude_shift_;
  double y_swap_phase_shift_;
  double y_swap_amplitude_;
  double y_swap_amplitude_shift_;
  double y_move_phase_shift_;
  double y_move_amplitude_;
  double y_move_amplitude_shift_;
  double z_swap_phase_shift_;
  double z_swap_amplitude_;
  double z_swap_amplitude_shift_;
  double z_move_phase_shift_;
  double z_move_amplitude_;
  double z_move_amplitude_shift_;
  double a_move_phase_shift_;
  double a_move_amplitude_;
  double a_move_amplitude_shift_;

  double pelvis_offset_;
  double pelvis_swing_;
  double hit_pitch_offset_;
  double arm_swing_gain_;

  bool ctrl_running_;
  bool real_running_;
  double time_;

  int phase_;
  double body_swing_y;
  double body_swing_z;
};

}



#endif /* OP3_WALKING_MODULE_H_ */