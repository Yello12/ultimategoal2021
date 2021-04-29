package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utils.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.firstinspires.ftc.teamcode.utils.Utils;

import java.util.function.DoubleSupplier;

import kotlin.Unit;

public class TurnToAngle extends CommandBase {

    private final double KP = 1.7;
    private final double KI = 0.02;
    private final double KD = 0.1;
    private final double KF = 0.0;
    private final double tolerance = 5 * Units.DEGREES;
    private final double time_tolerance = 0.1;
    private final double min_output = 0.2;
    private final double max_output = 1.0;
    private double start_time = Double.POSITIVE_INFINITY;
    private boolean is_at_before = false;

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
        double power = pidf.calculate(drive.getHeading(), 0.02);
//        dashboardTelemetry.addData("turn_pidf_power", power);
//        power = Utils.reverseClamp(power, -min_output, min_output);
//        dashboardTelemetry.addData("turn_pidf_power_clamped", power);
//        dashboardTelemetry.addData("turn_pidf_error", pidf.getError() * Units.TO_DEGREES);
        power = 0.4;
        drive.setOpenPower(-power, power);
//        dashboardTelemetry.update();

    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }


    @Override
    public boolean isFinished() {
        boolean is_at_now = pidf.atSetpoint();
//        if (!is_at_before && is_at_now) {
//            start_time = (double) System.nanoTime() / 1E9;
//        }
//        double elapsed = (double) System.nanoTime() / 1E9 - start_time;
//        dashboardTelemetry.addData("turn pidf time", elapsed);
//        if (elapsed >= time_tolerance && is_at_now) {
//            return true;
//        }
//        is_at_before = is_at_now;
        return is_at_now;
    }

}