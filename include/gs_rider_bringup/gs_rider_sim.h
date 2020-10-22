#pragma once
/**
 * @author Yuki Suga (ysuga@ysuga.net)
 * @date 2020/10/22
 */


#include "gs_rider_bringup/gs_rider_core.h"

namespace gs {

  class GSRiderSim : public GSRiderCore {
  private:
    double vx_;
    double alpha_;
    
    double moveDistance_;

  public:
  GSRiderSim():  vx_(0), alpha_(0), moveDistance_(0) {}
    virtual ~GSRiderSim() {}

  private:

    /**
     * @returns [m]
     */
    virtual double getAndResetMoveDistance() override {
      auto m = moveDistance_;
      moveDistance_ = 0;
      return m;
    }

    /**
     * @returns [radian]
     */
    virtual double getSteerAngle() const override { return alpha_; }

    virtual double getMoveVelocity() const override { return vx_; }

    /**
     * @param vx [m/sec]
     */
    virtual void setMoveVelocity(const double vx) override { vx_ = vx; }

    /**
     * @param alpha [radian]
     */
    virtual void setSteerAngle(const double alpha) override { alpha_ = alpha; }

    /**
     * @param dt [sec]
     */
    virtual void update(const double dt) override {
      moveDistance_ += vx_ * dt;
    }
  };

}
