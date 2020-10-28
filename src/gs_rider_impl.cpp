#include <ros/ros.h>


#include "gs_rider_bringup/gs_rider_core.h"




using namespace gs;


GSRiderImpl::GSRiderImpl(ros::NodeHandle* nh) : nodeHandle_(nh), l_count_(0), r_count_(0), init_l_count_(false), init_r_count_(false), distance_to_count_(0.003) {

  boost::function<void (const std_msgs::Int32&)> l_count_CB = [this] (const auto& msg) { 
    this->set_l_count(msg.data);
  };

  boost::function<void (const std_msgs::Int32&)> r_count_CB = [this] (const auto& msg) { 
    this->set_r_count(msg.data);
  };
  l_count_sub_ = nodeHandle_->subscribe<std_msgs::Int32>("esp32_row/wheel_lcount", 1000, l_count_CB);
  r_count_sub_ = nodeHandle_->subscribe<std_msgs::Int32>("esp32_row/wheel_lcount", 1000, r_count_CB);

  boost::function<void (const std_msgs::Int32&)> rc_ch1_CB = [this] (const auto& msg) { 
    this->set_rc_ch1(msg.data);
  };
  rc_ch1_sub_ = nodeHandle_->subscribe<std_msgs::Int32>("esp32_row/rc_ch1in", 1000, rc_ch1_CB);
}

GSRiderImpl::~GSRiderImpl() {}

/**
 * @returns [m]
 */
double GSRiderImpl::getAndResetMoveDistance() {
  // プロセス開始時に溜まっているカウントは無視する
  if (!init_) {
    l_count_prev_ = l_count_;
    r_count_prev_ = r_count_;
    init_ = true;
  }

  // 前回reset時との差分をとる
  l_count_diff_ = l_count_ - l_count_prev_;
  r_count_diff_ = r_count_ - r_count_prev_;

  // resetする
  l_count_prev_ = l_count_;
  r_count_prev_ = r_count_;

  // 移動距離を計算する
  return (l_count_diff_ + r_count_diff_) * distance_to_count_ / 2;
}

/**
 * @returns [radian]
 */
double GSRiderImpl::getSteerAngle() const {
  return 1522.0 - rc_ch1in_pwm_ / 450.0;
}

double GSRiderImpl::getMoveVelocity() const {
  return 0;
}

/**
 * @param vx [m/sec]
 */
void GSRiderImpl::setMoveVelocity(const double vx) {}

/**
 * @param alpha [radian]
 */
void GSRiderImpl::setSteerAngle(const double alpha) {}

/**
 * @param dt [sec]
 */
void GSRiderImpl::update(const double dt) {
}

