package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import com.acmerobotics.dashboard.FtcDashboard;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.android.dx.rop.cst.Constant;
import org.firstinspires.ftc.teamcode.commands.FlywheelJoystick;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.utils.MotorState;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Utils;

import kotlin.Unit;

public class Shooter extends SubsystemBase {

    public class FlyweelState{
        double back;
        double front;
        public FlyweelState(double _back, double _front){
            back = _back;
            front = _front;
        }
        public void set(double state){
            back = state;
            front = state;
        }
    }

    public enum Mode{
        Idle,
        Open,
        ClosedVelocity,
    }

    private final double CPR;
    private final double COUNTS_PER_RAD;
    private final double RAD_PER_COUNT;

    private final double MAX_OUTPUT = 0.5;

    private final Motor back_motor, front_motor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private MotorState back_state;
    private MotorState front_state;

    private FlyweelState desired_state;

    private Mode desired_mode;

    private double prev_timestamp;


    public Shooter(Motor _back_motor, Motor _front_motor, final double cpr) {

        back_motor = _back_motor;
        front_motor = _front_motor;

        CPR = cpr;
        COUNTS_PER_RAD = cpr / (2 * Math.PI);
        RAD_PER_COUNT = 1 / COUNTS_PER_RAD;

        back_motor.setDistancePerPulse(RAD_PER_COUNT);
        front_motor.setDistancePerPulse(RAD_PER_COUNT);

        desired_state = new FlyweelState(0,0);
        back_state = new MotorState();
        front_state = new MotorState();
    }

    public void initMotorControl(){
        back_motor.setFeedforwardCoefficients(Constants.BACK_MOTOR_KS, Constants.BACK_MOTOR_KV);
        back_motor.setVeloCoefficients(Constants.BACK_MOTOR_KP,Constants.BACK_MOTOR_KI,Constants.BACK_MOTOR_KD);
    }

    public void stop(){
        desired_mode = Mode.Idle;
        setMotorRunMode(Motor.RunMode.RawPower);
        desired_state.back = 0;
        desired_state.front = 0;
    }

    public void setOpenPower(double back, double front) {
        desired_mode = Mode.Open;
        setMotorRunMode(Motor.RunMode.RawPower);
        desired_state.back = back;
        desired_state.front = front;
    }

    public void setClosedVelocity(double back, double front) {
        desired_mode = Mode.ClosedVelocity;
        setMotorRunMode(Motor.RunMode.VelocityControl);
        desired_state.back = back;
        desired_state.front = front;
    }

    private void setMotorRunMode(Motor.RunMode mode){
        back_motor.setRunMode(mode);
        front_motor.setRunMode(mode);
    }

    public double getBackPower() {
        return back_motor.get();
    }
    public double getFrontPower() {
        return front_motor.get();
    }
    public MotorState getBackState() {
        return back_state;
    }
    public MotorState getFrontState() {
        return front_state;
    }
    public double getBackEncoderRaw() {
        return back_motor.encoder.getPosition();
    }
    public double getFrontEncoderRaw() {
        return front_motor.encoder.getPosition();
    }

    public void resetEncoders() {
        back_state = new MotorState();
        front_state = new MotorState();
        back_motor.encoder.reset();
        front_motor.encoder.reset();
    }

    public void addDashboardData(){
        dashboardTelemetry.addData("Back Desired", desired_state.back);
        dashboardTelemetry.addData("Back Power", getBackPower());
        dashboardTelemetry.addData("Back Encoder", getBackEncoderRaw());
        dashboardTelemetry.addData("Back Position", getBackState().position * Units.TO_DEGREES);
        dashboardTelemetry.addData("Back Velocity", getBackState().velocity * (Units.TO_ROTATIONS/Units.TO_MINUTES));
        dashboardTelemetry.addData("Back Encoder Velocity", back_motor.getCorrectedVelocity());

    }

    @Override
    public void periodic() {
        double cur_timestamp = (double) System.nanoTime() / 1E9;
        double dt = cur_timestamp - prev_timestamp;
        prev_timestamp = cur_timestamp;
        back_state.update(back_motor.encoder.getDistance(),dt);
        //front_state.update(front_motor.encoder.getDistance(),dt);

        if(desired_mode == Mode.Idle){
            back_motor.set(0);
            front_motor.set(0);
        } else if(desired_mode == Mode.Open){
            back_motor.set(desired_state.back);
            front_motor.set(desired_state.front);
        } else if (desired_mode == Mode.ClosedVelocity){
            back_motor.set(desired_state.back);
            front_motor.set(desired_state.front);
        }
    }


}
