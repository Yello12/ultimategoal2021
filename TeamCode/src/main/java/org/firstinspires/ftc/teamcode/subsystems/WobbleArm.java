package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Units;

import kotlin.Unit;

public class WobbleArm extends SubsystemBase {


    public enum Mode {
        Closed,
        Open,
    }

    private final double CLOSED_ANGLE = -90 * Units.DEGREES;
    private final double OPEN_ANGLE =  179 * Units.DEGREES;

    private final ServoEx wobble_claw;
    private final Motor arm_motor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Mode desired_mode;
    private double desired_output;

    private final double CPR = 530;
    private final double COUNTS_PER_RAD;
    private final double RAD_PER_COUNT;


    public WobbleArm(ServoEx _wobble_claw, Motor _arm_motor) {
        desired_output = 0;
        wobble_claw = _wobble_claw;
        arm_motor = _arm_motor;
        COUNTS_PER_RAD = CPR / (2 * Math.PI);
        RAD_PER_COUNT = 1 / COUNTS_PER_RAD;
        arm_motor.setDistancePerPulse(RAD_PER_COUNT);
        desired_mode = Mode.Closed;
    }

    public void close() {
        desired_mode = Mode.Closed;
    }
    public void open() {
        desired_mode = Mode.Open;
    }
    public double getAngle(){
        return arm_motor.getDistance();
    }
    public int getCounts(){
        return arm_motor.getCurrentPosition();
    }

    public void setOutput(double output){
        desired_output = output;
    }
    public void addDashboardData() {

    }

    @Override
    public void periodic() {
        arm_motor.set(desired_output);
        if (desired_mode == Mode.Closed) {
            wobble_claw.turnToAngle(CLOSED_ANGLE * Units.TO_DEGREES);
        } else if (desired_mode == Mode.Open) {
            wobble_claw.turnToAngle(OPEN_ANGLE * Units.TO_DEGREES);
        }
    }

}
