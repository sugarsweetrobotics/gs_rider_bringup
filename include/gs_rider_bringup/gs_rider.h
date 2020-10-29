#pragma once
/**
 * @author Yuki Suga (ysuga@ysuga.net)
 * @date 2020/10/22
 */

#include <math.h>
#include <memory>
#include <tuple>
#include "gs_rider_bringup/gs_rider_core.h"

namespace gs {

  class GSRider {
  private:
    const double L_;
    const std::shared_ptr<GSRiderCore> riderCore_;

  public:
  GSRider(const double L, const std::shared_ptr<GSRiderCore>& core): L_(L), riderCore_(core) {}
    ~GSRider() {}


  public:

    
    double getMoveVelocity() const { return riderCore_->getMoveVelocity(); }
    double getSteerAngle() const { return riderCore_->getSteerAngle(); }

    void setThrottleGain(const double gain) {
      riderCore_->setThrottleGain(gain);
    }
    
    void setVelocity(const double vx, const double vtheta) {
      const double alpha = vx != 0.0 ? asin(vtheta * L_ / vx) : 0.0;
      const double v = vx;
      riderCore_->setSteerAngle(alpha);
      riderCore_->setMoveVelocity(v);
    }

    auto getOdometry() {
      const double D = riderCore_->getAndResetMoveDistance();
      const double a = riderCore_->getSteerAngle();

      const double sina = sin(a); // this value is used many times.
      
      const double dtheta = D * sina / L_;
      const double dx = a != 0.0 ? L_ * sin(dtheta) / sina : D;
      const double dy = a != 0.0 ? L_ / sina - L_ * cos(dtheta) / sina : 0;
      return std::make_tuple(dx, dy, dtheta);
    }

    /**
     * @param dt [sec]
     * @returns Odometry (x, y, th)
     */
    auto update(const double dt) {
      riderCore_->update(dt);
      return getOdometry();
    }

  };


}
