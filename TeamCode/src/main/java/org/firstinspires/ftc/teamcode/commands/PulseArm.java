package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.robotcore.internal.webserver.WebObserver;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

public class PulseArm extends CommandBase {

    private WobbleArm wobble_arm;
    private double time_length;
    private double speed;
    private double init_timestamp;

    public PulseArm(WobbleArm _wobble_arm, double _time_length, double _speed) {
        wobble_arm = _wobble_arm;
        time_length = _time_length;
        speed = _speed;
    }

    @Override
    public void initialize() {
        init_timestamp = (double) System.nanoTime() / 1E9;
    }

    @Override
    public void execute() {
        wobble_arm.setOutput(speed);
    }

    @Override
    public boolean isFinished(){
        double cur_timestamp = (double) System.nanoTime() / 1E9;
        return (cur_timestamp - init_timestamp) > time_length;
    }

    @Override
    public void end(boolean interrupted){
        wobble_arm.setOutput(0.0);
    }
}
