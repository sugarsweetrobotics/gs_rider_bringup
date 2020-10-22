#pragma once



namespace gs {

  class OdometryAccumulator {
  private:
    double x_, y_, th_;
    
  public:
  OdometryAccumulator(const double init_x, const double init_y, const double init_theta) : x_(init_x), y_(init_y), th_(init_theta) {}

  public:
    auto push(const std::tuple<double, double, double>& d) {
      double dx, dy, dtheta;
      std::tie(dx, dy, dtheta) = d;
      const double C = cos(th_ + dtheta/2);
      const double S = sin(th_ + dtheta/2);
      x_ += dx * C - dy * S;
      y_ += dx * S + dy * C;
      th_ += dtheta;
      return std::make_tuple(x_, y_, th_);      
    }

    auto getPose() const {
      return std::make_tuple(x_, y_, th_);
    }

  };

}
