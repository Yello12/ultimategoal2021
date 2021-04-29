package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import java.util.function.DoubleSupplier;


public class ArmJoystick extends CommandBase {

    private final WobbleArm wobble_arm;
    private final DoubleSupplier throttle;
    private final double MAX_SPEED = 0.3;
    public ArmJoystick(WobbleArm _wobble_arm, DoubleSupplier _throttle) {
        wobble_arm = _wobble_arm;
        throttle = _throttle;
        addRequirements(wobble_arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        wobble_arm.setOutput(MAX_SPEED*throttle.getAsDouble());
    }

}