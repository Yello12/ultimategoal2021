package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Utils;

public class TurnToAngle extends CommandBase {

    private final double KP = 0.5;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;
    private final double tolerance = Math.toRadians(5);
    private final double min_output = 0.15;
    private final double max_output = 0.5;

    private PIDF pidf;

    private Drive drive;
    private double angle;
    private boolean absolute;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public TurnToAngle(Drive subsystem, double _angle) {
        this(subsystem, _angle, false);
    }

    public TurnToAngle(Drive subsystem, double _angle, boolean _absolute) {
        absolute = _absolute;
        angle = _angle;
        drive = subsystem;
        pidf = new PIDF(KP, KI, KD, KF, -Math.PI, Math.PI);
        pidf.setOutputRange(-max_output, max_output);
        pidf.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        pidf.setCoefficients(Constants.TURN_KP, Constants.TURN_KI, Constants.TURN_KD, Constants.TURN_KF);
        if (!absolute) {
            angle += drive.getHeading();
            angle = Utils.angleRange(angle);
        }
        pidf.setSetpoint(angle);
    }

    @Override
    public void execute() {
        double power = pidf.calculate(drive.getHeading());
        dashboardTelemetry.addData("turn_pidf_power", power);
        power = Utils.reverseClamp(power, -min_output, min_output);
        dashboardTelemetry.addData("turn_pidf_power_clamped", power);
        dashboardTelemetry.addData("turn_pidf_error",Math.toDegrees(pidf.getError()));
        drive.setOpenPower(power, -power);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }


    @Override
    public boolean isFinished() {
        return pidf.atSetpoint();
    }

}