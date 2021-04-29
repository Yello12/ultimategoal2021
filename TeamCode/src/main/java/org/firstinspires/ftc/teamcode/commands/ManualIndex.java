package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;


public class ManualIndex extends CommandBase {

    private final Intake intake;
    private final Transport transport;

    private final DoubleSupplier throttle_left, throttle_right;
    private final BooleanSupplier suck, spit;

    public ManualIndex(Intake _intake, Transport _transport, DoubleSupplier _throttle_left, DoubleSupplier _throttle_right, BooleanSupplier _suck, BooleanSupplier _spit) {
        intake = _intake;
        transport = _transport;
        throttle_left = _throttle_left;
        throttle_right = _throttle_right;
        suck = _suck;
        spit = _spit;
        addRequirements(intake, transport);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if (suck.getAsBoolean()) {
            intake.suck();
        } else if (spit.getAsBoolean()) {
            intake.spit();
        } else {
            intake.stop();
        }
        transport.setStageOutput(0, throttle_right.getAsDouble());
        transport.setStageOutput(1,throttle_right.getAsDouble());
//        intake.set(throttle_left.getAsDouble(), throttle_right.getAsDouble());
    }

}