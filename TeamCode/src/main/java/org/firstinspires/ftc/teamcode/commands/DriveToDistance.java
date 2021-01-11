package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.utils.PIDF;
import org.firstinspires.ftc.teamcode.utils.Utils;
import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveToDistance extends CommandBase {

    private final double KP = 0.5;
    private final double KI = 0.0;
    private final double KD = 0.0;
    private final double KF = 0.0;
    private final double tolerance = 0.05;
    private final double min_output = 0.1;
    private final double max_output = 0.35;

    private PIDF pidf;

    private Drive drive;
    private double distance;



    public DriveToDistance(Drive subsystem, double _distance) {
        distance = _distance;
        drive = subsystem;
        pidf = new PIDF(KP, KI, KD, KF);
        pidf.setOutputRange(-max_output, max_output);
        pidf.setTolerance(tolerance);
    }

    @Override
    public void initialize() {
        pidf.setSetpoint(drive.getAverageEncoderDistance()+distance);
    }

    @Override
    public void execute() {
        double power = pidf.calculate(drive.getAverageEncoderDistance());
        power = Utils.reverseClamp(power,-min_output,min_output);
        drive.setOpenPower(power, power);
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