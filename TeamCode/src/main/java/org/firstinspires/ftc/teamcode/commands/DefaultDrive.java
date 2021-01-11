package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive;

import java.util.function.DoubleSupplier;

public class DefaultDrive extends CommandBase {

    private final Drive m_drive;
    private GamepadEx driver_gamepad;

    public DefaultDrive(Drive subsystem, GamepadEx _driver_gamepad) {
        m_drive = subsystem;
        GamepadEx driver_gamepad = _driver_gamepad;
        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        m_drive.tankDrive( driver_gamepad.getLeftY(), -driver_gamepad.getRightX());
    }

}