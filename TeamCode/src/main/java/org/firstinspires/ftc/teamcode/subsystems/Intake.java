package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.CRServo;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.kinematics.DifferentialOdometry;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.MotorState;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class Intake extends SubsystemBase {


    public enum Mode {
        Idle,
        Suck,
        Spit,
        Manual
    }

    private final double SUCK_ROLLER = 0.8;
    private final double SPIT_ROLLER = -0.8;

    private final double SUCK_BELT = 0.8;
    private final double SPIT_BELT = -0.8;

    private final Motor intake_roller;
    private final CRServo intake_belt;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Mode desired_mode;
    private double desired_belt = 0.0;
    private double desired_roller = 0.0;

    public Intake(Motor _intake_roller, CRServo _intake_belt) {
        intake_roller = _intake_roller;
        intake_belt = _intake_belt;
        desired_mode = Mode.Idle;
    }

    public void stop() {
        desired_mode = Mode.Idle;
    }

    public void suck() {
        desired_mode = Mode.Suck;
    }

    public void spit() {
        desired_mode = Mode.Spit;
    }

    public void set(double roller, double belt) {
        desired_mode = Mode.Manual;
        desired_roller = roller;
        desired_belt = belt;
    }

    public void addDashboardData() {
        dashboardTelemetry.addData("Intake Belt", desired_belt);
        dashboardTelemetry.addData("Intake Roller", desired_roller);
    }

    @Override
    public void periodic() {
        if (desired_mode == Mode.Idle) {
            intake_belt.set(0);
            intake_roller.set(0);
        } else if (desired_mode == Mode.Suck) {
            intake_belt.set(SUCK_BELT);
            intake_roller.set(SUCK_ROLLER);
        } else if (desired_mode == Mode.Spit) {
            intake_belt.set(SPIT_BELT);
            intake_roller.set(SPIT_ROLLER);
        } else if (desired_mode == Mode.Manual) {
            intake_belt.set(desired_belt);
            intake_roller.set(desired_roller);
        }
    }

}
