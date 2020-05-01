#ifndef PIDcontrol_h
#define PIDcontrol_h

struct PIDGain {
  double kp, ki, kd;
  PIDGain(double kp, double ki, double kd);
  PIDGain();
};

class PIDAxisController {
  PIDGain const og_gain_;
  PIDGain gain_;
  double i_max_{0}, i_min_{0} ;
  double d_state_{0}, i_state_{0}, d_term_{0}, i_term_{0};
  double add_gain_{0};
  double ctl_point_{0};

 public:
  PIDAxisController(PIDGain const &og_gain);
  void set_control_point(double const &ctl_point);
  void set_kp(double const &kp);
  void set_ki(double const &ki);
  void set_kd(double const &kd);
  void set_add_gain(double const &add); //band limited gain 0 to disable
  void set_i_max(double const &i_max);
  void set_i_min(double const &i_min);
  void set_i_limits(double const &i_limit);
  double get_kp() const;
  double get_ki() const;
  double get_kd() const;
  double get_add_gain() const;
  void reset_pid();
  void reset_i_state();
  
  double update_pid(double out_k, double position);
  double update_pid_bypass(double out_k, double measure_d);
};

struct PIDController {
    PIDAxisController x, y, z;
    PIDController(PIDGain const & og_gain_x, PIDGain const & og_gain_y, PIDGain const & og_gain_z);
};
#endif