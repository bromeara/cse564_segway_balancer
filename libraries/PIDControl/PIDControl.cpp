#include <PIDControl.h>
#include <Arduino.h>

PIDGain::PIDGain(double kp, double ki, double kd):kp(kp), ki(ki), kd(kd){}
PIDGain::PIDGain():kp(0), ki(0), kd(0){}

PIDController::PIDController(PIDGain const &og_gain_x, PIDGain const &og_gain_y,
                             PIDGain const &og_gain_z)
    : x(og_gain_x), y(og_gain_y), z(og_gain_z) {}

PIDAxisController::PIDAxisController(PIDGain const &og_gain)
    : og_gain_(og_gain), gain_(og_gain) {}

void PIDAxisController::set_control_point(double const &ctl_point){
    ctl_point_ = ctl_point;
}

void PIDAxisController::set_kp(double const &kp){
    gain_.kp = kp;
}

void PIDAxisController::set_ki(double const &ki){
    gain_.ki = ki;
}

void PIDAxisController::set_kd(double const &kd){
    gain_.kd = kd;
}

void PIDAxisController::set_add_gain(double const &add_gain){
    add_gain_ = add_gain;
}

void PIDAxisController::set_i_max(double const &i_max){
    if(i_max < i_min_){
        i_max_ = i_min_;
        i_min_ = i_max;
    } else {
        i_max_ = i_max;
    }
}

void PIDAxisController::set_i_min(double const &i_min){
    if(i_min > i_max_){
        i_min_ = i_max_;
        i_max_ = i_min;
    } else {
        i_min_ = i_min;
    }
}

void PIDAxisController::set_i_limits(double const &i_limit){
    i_min_ = - abs(i_limit);
    i_max_ = abs(i_limit);
}

double PIDAxisController::get_kp() const {
    return gain_.kp;
}

double PIDAxisController::get_ki() const {
    return gain_.ki;
}

double PIDAxisController::get_kd() const {
    return gain_.kd;
}

double PIDAxisController::get_add_gain() const{
    return add_gain_;
}

void PIDAxisController::reset_pid(){
    gain_ = og_gain_;
}

void PIDAxisController::reset_i_state(){
    i_state_ = 0;
}

// Use when estimating the derivative
double PIDAxisController::update_pid(double out_k, double position){
    // Integrate the new error value
    auto error = ctl_point_ - out_k;
    i_state_ += error;

    // Apply anti-windup
    if(i_state_ > i_max_){i_state_ = i_max_;}
    else if (i_state_ < i_min_) {i_state_ = i_min_;}

    // Calculate integral component
    i_term_ = gain_.ki * i_state_;

    d_term_ = add_gain_ * (d_state_ - position);
    d_state_ = d_term_ + position;

    return i_term_ + gain_.kd * d_term_ + gain_.kp * error;
}

// Use when you can measure the derivative
double PIDAxisController::update_pid_bypass(double out_k, double measure_d){
    // Integrate the new error value
    auto error = ctl_point_ - out_k;
    i_state_ += error;

    // Apply anti-windup
    if(i_state_ > i_max_){i_state_ = i_max_;}
    else if (i_state_ < i_min_) {i_state_ = i_min_;}

    // Calculate integral component
    i_term_ = gain_.ki * i_state_;

    return i_term_ + gain_.kd * measure_d + gain_.kp * error;
}