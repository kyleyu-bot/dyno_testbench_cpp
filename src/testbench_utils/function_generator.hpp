#pragma once

#include <random>

namespace testbench_utils {

enum class FunctionGeneratorMode {
    OFF,
    DC,
    SINE,
    SQUARE,
    TRIANGLE,
    SAWTOOTH,
    WHITE_NOISE,
    CHIRP_LINEAR,
};

class FunctionGenerator {
public:
    FunctionGenerator();

    // Advance internal time by dt (seconds — pass actual measured cycle time).
    // Recomputes cached value, derivative, and integral. No-op when stopped.
    void update(double dt);

    // Zero ALL state and ALL parameters, clear stopped flag, mode → OFF.
    void reset();

    // Pause waveform output.
    //   full_reset = false : freeze — output holds at last value, all parameters preserved.
    //   full_reset = true  : calls reset(), zeros everything.
    void stop(bool full_reset = false);

    double getValue()         const { return value_; }
    double getValueDot()      const { return value_dot_; }
    double getValueDDot()     const { return value_ddot_; }
    double getValueIntegral() const { return integral_; }

    void                  setMode(FunctionGeneratorMode m) { mode_ = m; }
    FunctionGeneratorMode getMode()                  const { return mode_; }

    void   setAmplitude(double v) { amplitude_ = v; }
    double getAmplitude()   const { return amplitude_; }

    void   setFrequency(double v) { frequency_ = v; }
    double getFrequency()   const { return frequency_; }

    // Offset: center of oscillation for SINE/SQUARE/TRIANGLE; base (minimum) for SAWTOOTH.
    void   setOffset(double v) { offset_ = v; }
    double getOffset()   const { return offset_; }

    void   setPhase(double v) { phase_ = v; }  // radians
    double getPhase()   const { return phase_; }

    // Auto-stop after N seconds of running. 0 = never.
    void setResetTime(double v) { reset_time_ = v; }

    void setChirpLowFrequency(double v)  { chirp_f_low_  = v; }
    void setChirpHighFrequency(double v) { chirp_f_high_ = v; }
    void setChirpDuration(double v)      { chirp_dur_    = v; }

private:
    FunctionGeneratorMode mode_       = FunctionGeneratorMode::OFF;
    double amplitude_   = 1.0;
    double frequency_   = 1.0;
    double offset_      = 0.0;
    double phase_       = 0.0;
    double reset_time_  = 0.0;
    double chirp_f_low_ = 0.1;
    double chirp_f_high_= 10.0;
    double chirp_dur_   = 10.0;

    double t_elapsed_   = 0.0;
    double value_       = 0.0;
    double value_dot_   = 0.0;
    double value_ddot_  = 0.0;
    double prev_value_  = 0.0;
    double prev_dot_    = 0.0;
    double integral_    = 0.0;
    bool   stopped_     = false;

    std::mt19937                          rng_;
    std::uniform_real_distribution<double> noise_dist_{-1.0, 1.0};

    void recompute(double dt);
};

} // namespace testbench_utils
