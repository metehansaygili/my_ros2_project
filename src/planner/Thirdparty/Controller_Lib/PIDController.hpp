#pragma once

#ifndef PID_CONTROLLER_HPP
#define PID_CONTROLLER_HPP

#include <deque>
#include <numeric>
#include <cmath>
#include <cstddef>

class PIDController
{
public:
    PIDController()
    {
        reset();
    }

    /* ===== Public API ===== */
    double compute(
        double error, // steering_error or control error
        double dt     // sampling time [s]
    )
    {
        /* ===== Integral (windowed) ===== */
        integral_error_window.push_back(error);
        if (integral_error_window.size() > integral_error_window_size)
        {
            integral_error_window.pop_front();
        }

        double integral_error =
            std::accumulate(integral_error_window.begin(),
                            integral_error_window.end(),
                            0.0);

        /* ===== Derivative ===== */
        double derivative = (error - prev_error) / dt;

        // Spike rejection
        if (std::abs(derivative) > derivative_limit)
        {
            derivative = 0.0;
        }

        /* ===== PID Output ===== */
        double control =
            kp * error +
            ki * integral_error +
            kd * derivative;

        prev_error = error;

        int raw_steering = static_cast<int>(((-control / 180.0) + 0.5) * 3600);

        // Limit the steering command to the range [0, 3600].
        if (raw_steering > 3600)
        {
            raw_steering = 3600;
        }
        else if (raw_steering < 0)
        {
            raw_steering = 0;
        }

        // TODO: Mapping will be change
        raw_steering = 3600 - raw_steering; // current mapping

        return raw_steering;
    }

    /* ===== Stateless compute for prediction (doesn't update internal state) ===== */
    double computeStateless(
        double error,
        double dt,
        const std::deque<double>& external_integral_window,
        double external_prev_error
    ) const
    {
        /* ===== Integral (from external window) ===== */
        double integral_error =
            std::accumulate(external_integral_window.begin(),
                            external_integral_window.end(),
                            0.0);

        /* ===== Derivative ===== */
        double derivative = (error - external_prev_error) / dt;

        // Spike rejection
        if (std::abs(derivative) > derivative_limit)
        {
            derivative = 0.0;
        }

        /* ===== PID Output ===== */
        double control =
            kp * error +
            ki * integral_error +
            kd * derivative;

        int raw_steering = static_cast<int>(((-control / 180.0) + 0.5) * 3600);

        // Limit the steering command to the range [0, 3600].
        if (raw_steering > 3600)
        {
            raw_steering = 3600;
        }
        else if (raw_steering < 0)
        {
            raw_steering = 0;
        }

        // TODO: Mapping will be change
        raw_steering = 3600 - raw_steering; // current mapping

        return raw_steering;
    }

    /* ===== State getters for prediction ===== */
    const std::deque<double>& getIntegralWindow() const
    {
        return integral_error_window;
    }

    double getPrevError() const
    {
        return prev_error;
    }

    /* ===== Parameter setters ===== */
    void setGains(double kp_, double ki_, double kd_)
    {
        kp = kp_;
        ki = ki_;
        kd = kd_;
    }

    void setDerivativeLimit(double limit)
    {
        derivative_limit = limit;
    }

    void reset()
    {
        prev_error = 0.0;
        integral_error_window.clear();
        integral_error_window.resize(integral_error_window_size, 0.0);
    }

private:
    /* ===== Gains ===== */
    double kp = 1.3;
    double ki = 0.05;
    double kd = 0.1;

    /* ===== Internal state ===== */
    double prev_error = 0.0;

    /* ===== Integral window ===== */
    static constexpr std::size_t integral_error_window_size = 20;
    std::deque<double> integral_error_window =
        std::deque<double>(integral_error_window_size, 0.0);

    /* ===== Safety ===== */
    double derivative_limit = 100.0;
};

#endif // PID_CONTROLLER_HPP