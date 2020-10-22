#pragma once
/**
 * @author Yuki Suga (ysuga@ysuga.net)
 * @date 2020/10/22
 */


namespace gs {

  class GSRiderCore {
  public:

    virtual ~GSRiderCore() {}

  public:
    /**
     * @returns [m]
     */
    virtual double getAndResetMoveDistance() = 0;

    /**
     * @returns [radian]
     */
    virtual double getSteerAngle() const = 0;

    virtual double getMoveVelocity() const = 0;

    /**
     * @param vx [m/sec]
     */
    virtual void setMoveVelocity(const double vx) = 0;

    /**
     * @param alpha [radian]
     */
    virtual void setSteerAngle(const double alpha) = 0;

    /**
     * @param dt [sec]
     */
    virtual void update(const double dt) = 0;
  };


}
