#include "testbench_utils/function_generator.hpp"

#include <cmath>

namespace testbench_utils {

static constexpr double TWO_PI = 2.0 * M_PI;

FunctionGenerator::FunctionGenerator()
    : rng_(std::random_device{}())
{}

void FunctionGenerator::reset()
{
    waveform_type_ = WaveformType::OFF;
    control_type_  = ControlType::NONE;
    amplitude_   = 0.0;
    frequency_   = 1.0;
    offset_      = 0.0;
    phase_       = 0.0;
    reset_time_  = 0.0;
    chirp_f_low_ = 0.1;
    chirp_f_high_= 10.0;
    chirp_dur_   = 10.0;

    t_elapsed_   = 0.0;
    value_       = 0.0;
    value_dot_   = 0.0;
    value_ddot_  = 0.0;
    prev_value_  = 0.0;
    prev_dot_    = 0.0;
    integral_    = 0.0;
    stopped_     = false;
}

void FunctionGenerator::enable()
{
    t_elapsed_  = 0.0;
    value_      = 0.0;
    value_dot_  = 0.0;
    value_ddot_ = 0.0;
    prev_value_ = 0.0;
    prev_dot_   = 0.0;
    integral_   = 0.0;
    stopped_    = false;
}

void FunctionGenerator::stop(bool full_reset)
{
    if (full_reset) {
        reset();
    } else {
        stopped_    = true;
        value_      = 0.0;
        value_dot_  = 0.0;
        value_ddot_ = 0.0;
    }
}

void FunctionGenerator::update(double dt)
{
    if (stopped_) return;

    if (reset_time_ > 0.0 && t_elapsed_ >= reset_time_) {
        value_      = 0.0;
        value_dot_  = 0.0;
        value_ddot_ = 0.0;
        return;
    }

    t_elapsed_ += dt;
    recompute(dt);
    integral_ += value_ * dt;
}

void FunctionGenerator::recompute(double dt)
{
    const double A = amplitude_;
    const double f = frequency_;
    const double C = offset_;
    const double t = t_elapsed_;

    switch (waveform_type_) {
    case WaveformType::OFF:
        value_      = 0.0;
        value_dot_  = 0.0;
        value_ddot_ = 0.0;
        break;

    case WaveformType::DC:
        value_      = C;
        value_dot_  = 0.0;
        value_ddot_ = 0.0;
        break;

    case WaveformType::SINE: {
        const double theta = TWO_PI * f * t + phase_;
        const double wn    = TWO_PI * f;
        value_      = A * std::sin(theta) + C;
        value_dot_  = A * wn * std::cos(theta);
        value_ddot_ = -A * wn * wn * std::sin(theta);
        break;
    }

    case WaveformType::SQUARE: {
        const double s     = std::sin(TWO_PI * f * t + phase_);
        const double sign  = (s >= 0.0) ? 1.0 : -1.0;
        value_      = C + A * sign;
        value_dot_  = 0.0;
        value_ddot_ = 0.0;
        break;
    }

    case WaveformType::TRIANGLE: {
        const double tau = std::fmod(f * t + phase_ / TWO_PI, 1.0);
        const double raw = 2.0 * std::abs(2.0 * tau - 1.0) - 1.0;
        value_      = C + A * raw;
        value_dot_  = (tau < 0.5) ? -4.0 * A * f : 4.0 * A * f;
        value_ddot_ = 0.0;
        break;
    }

    case WaveformType::SAWTOOTH: {
        const double tau = std::fmod(f * t + phase_ / TWO_PI, 1.0);
        value_      = C + A * tau;
        value_dot_  = A * f;
        value_ddot_ = 0.0;
        break;
    }

    case WaveformType::WHITE_NOISE: {
        prev_dot_   = value_dot_;
        prev_value_ = value_;
        value_      = C + A * noise_dist_(rng_);
        value_dot_  = (dt > 0.0) ? (value_ - prev_value_) / dt : 0.0;
        value_ddot_ = (dt > 0.0) ? (value_dot_ - prev_dot_)  / dt : 0.0;
        break;
    }

    case WaveformType::CHIRP_LINEAR: {
        if (t > chirp_dur_) {
            value_      = C;
            value_dot_  = 0.0;
            value_ddot_ = 0.0;
            break;
        }
        const double T       = chirp_dur_;
        const double f_lo    = chirp_f_low_;
        const double f_hi    = chirp_f_high_;
        const double inst_f  = f_lo + (f_hi - f_lo) * t / T;
        const double phi     = TWO_PI * (f_lo * t + (f_hi - f_lo) * t * t / (2.0 * T)) + phase_;
        const double wn      = TWO_PI * inst_f;
        const double wn_dot  = TWO_PI * (f_hi - f_lo) / T;
        value_      = A * std::sin(phi) + C;
        value_dot_  = A * wn * std::cos(phi);
        value_ddot_ = A * (wn_dot * std::cos(phi) - wn * wn * std::sin(phi));
        break;
    }
    }
}

} // namespace testbench_utils
