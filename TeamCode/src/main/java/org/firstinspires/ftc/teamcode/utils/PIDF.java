package org.firstinspires.ftc.teamcode.utils;

public class PIDF {

    private double kp, ki, kd, kf;
    private double min_in;
    private double max_in;
    private double min_out;
    private double max_out;
    private boolean continuous;

    private double last_error;
    private double cur_error;
    private double error_tolerance = 0.05;

    private double last_timestamp;

    private double proportion;
    private double integral;
    private double derivative;

    private double setpoint;
    private double output;

    public PIDF(double _kp, double _ki, double _kd, double _kf) {
        this(_kp, _ki, _kd, _kf, 0, 0);
    }

    public PIDF(double _kp, double _ki, double _kd, double _kf, double _min_in, double _max_in) {
        kp = _kp;
        ki = _ki;
        kd = _kd;
        kf = _kf;

        min_in = _min_in;
        max_in = _max_in;
        continuous = min_in < max_in;

        min_out = Double.NEGATIVE_INFINITY;
        max_out = Double.POSITIVE_INFINITY;

        last_error = 0;
        cur_error = 0;
        last_timestamp = 0;

        proportion = 0;
        integral = 0;
        derivative = 0;

        setpoint = 0;
        output = 0;
    }

    public void reset() {
        last_error = 0;
        cur_error = 0;

        proportion = 0;
        integral = 0;
        derivative = 0;

        setpoint = 0;
        output = 0;
    }

    public void setCoefficients(double _kp, double _ki, double _kd, double _kf){
        kp = _kp;
        ki = _ki;
        kd = _kd;
        kf = _kf;
        reset();
    }


    public void setTolerance(double tolerance) {
        error_tolerance = tolerance;
    }

    public void setOutputRange(double _min_out, double _max_out) {
        min_out = _min_out;
        max_out = _max_out;
    }

    public void setSetpoint(double _setpoint) {
        reset();
        setpoint = _setpoint;
    }

    public boolean atSetpoint() {
        return Math.abs(cur_error) <= error_tolerance;
    }


    public double getError() {
        return cur_error;
    }
    public double calculate(double input){
        double cur_timestamp = (double) System.nanoTime() / 1E9;

        if (last_timestamp == 0) {
            last_timestamp = cur_timestamp;
        }
        double dt = cur_timestamp - last_timestamp;

        last_timestamp = cur_timestamp;
        return calculate(input, dt);
    }

    public double calculate(double input, double dt) {
        if (dt < 1E-6) {
            dt = 1E-6;
        }

        cur_error = setpoint - input;
        double error_change = cur_error - last_error;
        last_error = cur_error;

        if (continuous && Math.abs(cur_error) > (max_in - min_in) / 2) {
            if (cur_error > 0) {
                cur_error -= max_in - min_in;
            } else {
                cur_error += max_in - min_in;
            }
        }
        proportion = cur_error;
        integral += cur_error * dt;
        derivative = error_change / dt;

        output = (kp * proportion) + (ki * integral) + (kd * derivative) + (kf * setpoint);
        output = Utils.clamp(output, min_out, max_out);
        return output;

    }

}

