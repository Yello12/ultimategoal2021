package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.drivebase.DifferentialDrive;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor.Encoder;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.concurrent.CountedCompleter;

public class DriveSubsystem extends SubsystemBase {

    private final DifferentialDrive diff_drive;

    private final MotorEx left_motor, right_motor;
    private final GyroEx imu;

    private final double WHEEL_DIAMETER;
    private final double WHEEL_CIRCUMFERENCE;

    private final double CPR;
    private final double COUNTS_PER_RAD;
    private final double RAD_PER_COUNT;
    private final double COUNTS_PER_METER;
    private final double METERS_PER_COUNT;

    public DriveSubsystem(MotorEx _left_motor, MotorEx _right_motor, GyroEx _imu, final double diameter, final double cpr) {
        left_motor = _left_motor;
        right_motor = _right_motor;
        imu = _imu;

        WHEEL_DIAMETER = diameter;
        WHEEL_CIRCUMFERENCE = 2 * Math.PI;

        CPR = cpr;
        COUNTS_PER_RAD = cpr / (2 * Math.PI);
        RAD_PER_COUNT = 1 / COUNTS_PER_RAD;

        COUNTS_PER_METER = CPR / WHEEL_CIRCUMFERENCE;
        METERS_PER_COUNT = 1 / COUNTS_PER_METER;

        diff_drive = new DifferentialDrive(left_motor, right_motor);
    }

    public void arcadeDrive(double fwd, double rot) {
        diff_drive.arcadeDrive(fwd, rot);
    }

    public double getLeftEncoderRaw() {
        return left_motor.encoder.getPosition();
    }

    public double getLeftEncoderDistance() {
        return left_motor.encoder.getRevolutions() * METERS_PER_COUNT;
    }

    public double getRightEncoderRaw() {
        return right_motor.encoder.getPosition();
    }

    public double getRightEncoderDistance() {
        return right_motor.encoder.getRevolutions() * METERS_PER_COUNT;
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

    public double getHeading() {
        return imu.getHeading();
    }

    public void resetHeading() {
        imu.reset();
    }


    public void resetEncoders() {
        left_motor.encoder.reset();
        right_motor.encoder.reset();
    }


}
