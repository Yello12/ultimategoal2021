package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.util.Range;


public class Chassis {
    private int LEFT_CPR = 530;
    private int RIGHT_CPR = 530;
    private double LEFT_ENC_PER_RAD = LEFT_CPR / (2*Math.PI);
    private double RIGHT_ENC_RAD_RAD =  RIGHT_CPR / (2*Math.PI);
    private double LEFT_RAD_PER_ENC = 1 / LEFT_ENC_PER_RAD;
    private double RIGHT_RAD_PER_ENC = 1 / RIGHT_ENC_RAD_RAD;

    private boolean LEFT_REVERSED = false;
    private boolean RIGHT_REVERSED = true;
    private boolean ENCODER_REVERSED = true;

    private double MAX_POWER = 0.3;
    private int GAMEPAD_EXPONENT = 3;

    private BNO055IMU imu = null;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;
    private double leftDesiredPower = 0;
    private double rightDesiredPower = 0;

    public Chassis(DcMotor _leftDrive, DcMotor _rightDrive, BNO055IMU _imu) {
        imu = _imu;
        leftDrive = _leftDrive;
        rightDrive = _rightDrive;
    }

    public void init(){
        leftDrive.setDirection(LEFT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD );
        rightDrive.setDirection(RIGHT_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
    }
    public void resetEncoders() {
        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void stop() {
        setPower(0, 0);
    }

    public void setPower(double left, double right) {
        leftDesiredPower = Range.clip(left, -MAX_POWER, MAX_POWER);
        rightDesiredPower = Range.clip(right, -MAX_POWER, MAX_POWER);

    }

    public void setPowerFromGamepad(double throttle, double rotation) {
        double left = Math.pow(throttle-rotation,GAMEPAD_EXPONENT);
        double right = Math.pow(throttle+rotation,GAMEPAD_EXPONENT);
        setPower(left, right);
    }
    private int getLeftPositionRaw(){
        return (ENCODER_REVERSED ? -1 : 1) * leftDrive.getCurrentPosition();
    }
    private int getRightPositionRaw(){
        return (ENCODER_REVERSED ? -1 : 1) * rightDrive.getCurrentPosition();
    }
    public double getLeftPosition(){
        return getLeftPositionRaw() * LEFT_RAD_PER_ENC;
    }
    public double getRightPosition(){
        return getRightPositionRaw() * RIGHT_RAD_PER_ENC;
    }
//    public void updateTelemetry() {
//        telemetry.addData("Left Encoder", leftDrive.getCurrentPosition());
//        telemetry.addData("Right Encoder", rightDrive.getCurrentPosition());
//        telemetry.update();
//    }

    public void onLoop() {
        leftDrive.setPower(leftDesiredPower);
        rightDrive.setPower(rightDesiredPower);
//        updateTelemetry();
    }

}
