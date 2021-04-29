package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final Drive drive;
    private GamepadEx gamepad;

    public DefaultDrive(Drive _drive, GamepadEx _gamepad) {
        drive = _drive;
        gamepad = _gamepad;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.tankDrive(-gamepad.getLeftY(), gamepad.getRightX());

    }

}