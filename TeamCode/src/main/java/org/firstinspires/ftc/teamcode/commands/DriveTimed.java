package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

public class DriveTimed extends CommandBase {

    private Drive drive;
    private double time_length;
    private double speed;
    private double init_timestamp;

    public DriveTimed(Drive _drive, double _time_length, double _speed) {
        drive = _drive;
        time_length = _time_length;
        speed = _speed;
    }

    @Override
    public void initialize() {
        init_timestamp = (double) System.nanoTime() / 1E9;
    }

    @Override
    public void execute() {
        drive.setOpenPower(speed,speed);
    }

    @Override
    public boolean isFinished(){
        double cur_timestamp = (double) System.nanoTime() / 1E9;
        return (cur_timestamp - init_timestamp) < time_length;
    }

    @Override
    public void end(boolean interrupted){
        drive.stop();
    }
}
