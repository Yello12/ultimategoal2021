package org.firstinspires.ftc.teamcode.utils;

import com.arcrobotics.ftclib.hardware.motors.Motor;

public class MotorState {
    public double position;
    public double prev_position;
    public double velocity;
    public double prev_velocity;
    public double acceleration;

    public MotorState() {
        this(0, 0, 0, 0, 0);
    }

    public MotorState(double _position, double _prev_position, double _velocity, double _prev_velocity, double _acceleration
    ) {
        position = _position;
        prev_position = _prev_position;
        velocity = _velocity;
        prev_velocity = _prev_velocity;
        acceleration = _acceleration;
    }

    public void update(double _position, double dt) {
        position = _position;
        velocity = (position - prev_position) / dt;
        acceleration = (velocity - prev_velocity) / dt;
        prev_position = position;
        prev_velocity = velocity;
    }
}
