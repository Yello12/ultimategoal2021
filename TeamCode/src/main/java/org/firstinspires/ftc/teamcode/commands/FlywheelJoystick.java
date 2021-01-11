package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.function.DoubleSupplier;


public class FlywheelJoystick extends CommandBase {

    private final Shooter shooter;
    private final DoubleSupplier throttle;

    public FlywheelJoystick(Shooter _shooter, DoubleSupplier _throttle) {
        shooter = _shooter;
        throttle = _throttle;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetEncoders();
    }

    @Override
    public void execute() {
        shooter.setOpenPower(throttle.getAsDouble(), throttle.getAsDouble());
    }

}