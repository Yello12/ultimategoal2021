package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Main TeleOp")
public class TeleopCommand extends CommandOpMode {

    static final double WHEEL_DIAMETER = 0.09;
    static final double DRIVE_CPR = 530;

    private MotorEx left_drive, right_drive;
    private GyroEx imu;

    private DriveSubsystem drive;
    private GamepadEx driver_gamepad;
    private DefaultDrive drive_command;

    private Button turn_button;

    @Override
    public void initialize() {
        left_drive = new MotorEx(hardwareMap, "left_drive");
        right_drive = new MotorEx(hardwareMap, "right_drive");
        imu = new RevIMU(hardwareMap,"imu");

        drive = new DriveSubsystem(left_drive, right_drive, imu, WHEEL_DIAMETER, DRIVE_CPR);

        driver_gamepad = new GamepadEx(gamepad1);
        drive_command = new DefaultDrive(drive, () -> driver_gamepad.getLeftY(), () -> driver_gamepad.getLeftX());

        drive.setDefaultCommand(drive_command);
    }

}