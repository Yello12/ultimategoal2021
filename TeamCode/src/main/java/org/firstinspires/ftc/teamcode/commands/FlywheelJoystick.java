package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.utils.Constants;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class FlywheelJoystick extends CommandBase {

    private final Transport transport;
    private final Shooter shooter;

    private final DoubleSupplier throttle_transport;
    private final DoubleSupplier throttle_shoot;
    private final BooleanSupplier shoot_button;
    private final BooleanSupplier shoot_reverse;

    public FlywheelJoystick(Transport _transport, Shooter _shooter, DoubleSupplier _throttle_transport, DoubleSupplier _throttle_shoot, BooleanSupplier _shoot_button, BooleanSupplier _shooter_reverse) {
        transport = _transport;
        shooter = _shooter;
        throttle_transport = _throttle_transport;
        throttle_shoot = _throttle_shoot;
        shoot_button = _shoot_button;
        shoot_reverse = _shooter_reverse;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        shooter.resetEncoders();
    }

    @Override
    public void execute() {
        if (shoot_button.getAsBoolean()) {
            shooter.setOpenPower(1.0, 1.0);
        } else if(shoot_reverse.getAsBoolean()) {
            shooter.setOpenPower(-0.2, -0.2);

        }
        else{
            shooter.stop();
        }

//        transport.setStageOutput(0, throttle_transport.getAsDouble());
//        transport.setStageOutput(1, throttle_transport.getAsDouble());
//        shooter.setOpenPower(throttle_shoot.getAsDouble(), throttle_shoot.getAsDouble());
    }

}