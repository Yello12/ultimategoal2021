package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorState;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Drive extends SubsystemBase {

    public class WheelState{
        double left;
        double right;
        public WheelState(double _left, double _right){
            left = _left;
            right = _right;
        }
        public void norm(double max){
            double max_output = Math.max(Math.abs(left),Math.abs(right));
            if(max_output == 0){
                return;
            }
            double scale = max / max_output;
            left *= scale;
            right *= scale;
        }
    }

    public enum Mode{
        Idle,
        Open,
        ClosedVelocity,
        ClosedPosition,
    }

    private final double WHEEL_DIAMETER;
    private final double WHEEL_CIRCUMFERENCE;

    private final double CPR;
    private final double COUNTS_PER_RAD;
    private final double RAD_PER_COUNT;
    private final double COUNTS_PER_METER;
    private final double METERS_PER_COUNT;

    private final double MAX_OUTPUT = 0.5;

    private final Motor left_motor, right_motor;
    private final GyroEx imu;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private MotorState left_state;
    private MotorState right_state;

    private WheelState desired_state;
//    private WheelState feedforward;

    private Mode desired_mode;


    public Drive(Motor _left_motor, Motor _right_motor, GyroEx _imu, final double diameter, final double cpr) {

        left_motor = _left_motor;
        right_motor = _right_motor;
        imu = _imu;

        WHEEL_DIAMETER = diameter;
        WHEEL_CIRCUMFERENCE = diameter * Math.PI;

        CPR = cpr;
        COUNTS_PER_RAD = cpr / (2 * Math.PI);
        RAD_PER_COUNT = 1 / COUNTS_PER_RAD;

        COUNTS_PER_METER = CPR / WHEEL_CIRCUMFERENCE;
        METERS_PER_COUNT = 1 / COUNTS_PER_METER;

        left_motor.setDistancePerPulse(METERS_PER_COUNT);
        right_motor.setDistancePerPulse(METERS_PER_COUNT);

        desired_mode = Mode.Idle;
        desired_state = new WheelState(0,0);
    }

    public void tankDrive(double throttle, double rotation) {
        double left = throttle + rotation;
        double right = throttle - rotation;
        setOpenPower(left, right);
    }

    public void stop(){
        desired_mode = Mode.Idle;
        setMotorRunMode(Motor.RunMode.RawPower);
        desired_state.left = 0;
        desired_state.right  = 0;
    }

    public void setOpenPower(double left, double right) {
        desired_mode = Mode.Open;
        setMotorRunMode(Motor.RunMode.RawPower);
        desired_state.left = left;
        desired_state.right  = right;
        desired_state.norm(MAX_OUTPUT);
    }

    public void setClosedVelocity(double left, double right) {
        desired_mode = Mode.ClosedVelocity;
        setMotorRunMode(Motor.RunMode.VelocityControl);
        desired_state.left = left;
        desired_state.right  = right;
//        desired_state.norm(MAX_VELOCITY);
    }

    private void setMotorRunMode(Motor.RunMode mode){
        left_motor.setRunMode(mode);
        right_motor.setRunMode(mode);
    }

    public double getLeftEncoderRaw() {
        return left_motor.encoder.getPosition();
    }

    public double getLeftEncoderDistance() {
        return left_motor.encoder.getDistance();
    }

    public double getRightEncoderRaw() {
        return -right_motor.encoder.getPosition();
    }

    public double getRightEncoderDistance() {
        return -right_motor.encoder.getDistance();
    }

    public double getAverageEncoderDistance() {
        return (getLeftEncoderDistance() + getRightEncoderDistance()) / 2.0;
    }

    public double getHeading() {
        return Utils.angleRange(getAbsoluteHeading());
    }

    public double getAbsoluteHeading() {
        return Math.toRadians(imu.getHeading());
    }

    public double getLeftPower() {
        return left_motor.get();
    }

    public double getRightPower() {
        return right_motor.get();
    }

    public void resetHeading() {
        imu.reset();
    }


    public void resetEncoders() {
        left_motor.encoder.reset();
        right_motor.encoder.reset();
    }
    
    public void addDashboardData(){
        dashboardTelemetry.addData("Left Power", getLeftPower());
        dashboardTelemetry.addData("Right Power", getRightPower());
        dashboardTelemetry.addData("Heading", getHeading());
        dashboardTelemetry.addData("Heading Absolute", getAbsoluteHeading());
        dashboardTelemetry.addData("Left Ticks", getLeftEncoderRaw());
        dashboardTelemetry.addData("Right Ticks", getRightEncoderRaw());
        dashboardTelemetry.addData("Left Distance", getLeftEncoderDistance());
        dashboardTelemetry.addData("Right Distance", getRightEncoderDistance());
        dashboardTelemetry.addData("Heading", getHeading() * Units.TO_DEGREES);
    }

    @Override
    public void periodic() {
        if(desired_mode == Mode.Idle){
            left_motor.set(0);
            right_motor.set(0);
        } else if(desired_mode == Mode.Open){
            left_motor.set(desired_state.left);
            right_motor.set(desired_state.right);
        } else if (desired_mode == Mode.ClosedVelocity){

        }
    }


}
