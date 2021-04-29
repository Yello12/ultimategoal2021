package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveToDistance extends CommandBase {


    private final double tolerance = 0.05;
    private final double speed;

    private PIDF pidf;

    private Drive drive;
    private double desired_distance;
    private double start_distance;
    private double start_left;
    private double start_right;

    private double TURN_KP = 1.0;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public DriveToDistance(Drive subsystem, double _distance, double _speed) {
        drive = subsystem;
        desired_distance = _distance;
        speed = _speed;
    }

    public double getLeft() {
        return drive.getLeftEncoderDistance() - start_left;
    }

    public double getRight() {
        return drive.getRightEncoderDistance() - start_right;
    }

    @Override
    public void initialize() {
        start_distance = drive.getAverageEncoderDistance();
        start_left = drive.getLeftEncoderDistance();
        start_right = drive.getRightEncoderDistance();
//        desired_distance += start_distance;
    }

    @Override
    public void execute() {
        double error = getLeft() - getRight();
        double turn_power = TURN_KP * error;
        drive.setOpenPower(speed - turn_power, speed + turn_power);
        dashboardTelemetry.addData("Distance", (getLeft()+getRight())/2.0);

    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }


    @Override
    public boolean isFinished() {
        double distance = (getLeft()+getRight())/2.0;
        return Math.abs(distance - desired_distance) <= tolerance;
    }

}