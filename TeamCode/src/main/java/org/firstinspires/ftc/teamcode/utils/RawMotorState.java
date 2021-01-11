package org.firstinspires.ftc.teamcode.utils;

public class RawMotorState {
    public double position;
    public double velocity;

    public RawMotorState() {
        this(0, 0);
    }

    public RawMotorState(double _position, double _velocity
    ) {
        position = _position;
        velocity = _velocity;
    }

}