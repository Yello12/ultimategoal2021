package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class AutoShoot extends CommandBase {

    private final Transport transport;
    private final Shooter shooter;
    private double init_timestamp;
    private double elapsed;

    public AutoShoot(Transport _transport, Shooter _shooter) {
        transport = _transport;
        shooter = _shooter;
        addRequirements(transport, shooter);

    }

    @Override
    public void initialize() {
        init_timestamp = (double) System.nanoTime() / 1E9;

    }

    @Override
    public void execute() {
        double timestamp = (double) System.nanoTime() / 1E9;
        elapsed = timestamp - init_timestamp;
        shooter.setOpenPower(1.0, 1.0);

        if (elapsed >= 1) {
            transport.setStageOutput(0, 0.2);
            transport.setStageOutput(1, 0.2);
        }
    }

    @Override
    public boolean isFinished() {
        return elapsed >= 7;
    }

    @Override
    public void end(boolean interrupted) {
        transport.stop();
        shooter.stop();
    }
}