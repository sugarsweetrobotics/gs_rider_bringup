#pragma once


#include <ros/ros.h>
#include "gs_rider_bringup/gs_rider_core.h"

namespace gs {


  class GSRiderImpl : public GSRiderCore {
  private:
    ros::NodeHandle *nodeHandle_;

    bool init_;
    int32_t l_count_;
    int32_t r_count_;
    int32_t l_count_prev_;
    int32_t r_count_prev_;
    int32_t l_count_diff_;
    int32_t r_count_diff_;
    double distance_to_count_;

    ros::Subscriber l_count_sub_;
    ros::Subscriber r_count_sub_;
    ros::Subscriber rc_ch1_sub_;

    uint16_t rc_ch1in_pwm_;
  public:
    GSRiderImpl(ros::NodeHandle* nh);
    virtual ~GSRiderImpl();

  public:
    void set_l_count(const int32_t count) { l_count_ = count; }
    void set_r_count(const int32_t count) { r_count_ = count; }
    void set_rc_ch1(const uint16_t count) { rc_ch1in_pwm_ = count; }
  public:
    /**
     * @returns [m]
     */
    virtual double getAndResetMoveDistance() override;

    /**
     * @returns [radian]
     */
    virtual double getSteerAngle() const override;

    virtual double getMoveVelocity() const override;

    /**
     * @param vx [m/sec]
     */
    virtual void setMoveVelocity(const double vx) override;

    /**
     * @param alpha [radian]
     */
    virtual void setSteerAngle(const double alpha) override;

    /**
     * @param dt [sec]
     */
    virtual void update(const double dt) override;
    

  };

}
