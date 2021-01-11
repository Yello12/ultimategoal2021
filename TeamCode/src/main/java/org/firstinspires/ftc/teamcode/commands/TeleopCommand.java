package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.utils.Units;

@TeleOp(name = "Main TeleOp")
public class TeleopCommand extends CommandOpMode {

    static final double WHEEL_DIAMETER = 0.09;
    static final double DRIVE_CPR = 530;
    static final double SHOOTER_CPR = 28;

    private Motor left_drive, right_drive, back_flywheel, front_flywheel;
    private GyroEx imu;

    private Drive drive;
    private Shooter shooter;

    private GamepadEx driver_gamepad;
    private Button turn_button;
    private Button dist_button;
    private Button time_drive_button;

    private DefaultDrive drive_command;
    private FlywheelJoystick flywheel_joystick;
    private DriveTimed time_drive;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initialize() {
        left_drive = new Motor(hardwareMap, "left_drive");
        right_drive = new Motor(hardwareMap, "right_drive");
        back_flywheel = new Motor(hardwareMap, "back_flywheel");
        front_flywheel = new Motor(hardwareMap, "front_flywheel");

        imu = new RevIMU(hardwareMap, "imu");
        imu.init();

        drive = new Drive(left_drive, right_drive, imu, WHEEL_DIAMETER, DRIVE_CPR);
        shooter = new Shooter(back_flywheel, front_flywheel, SHOOTER_CPR);
        drive_timed = new DriveTimed(drive, 5, 1);

        driver_gamepad = new GamepadEx(gamepad1);
        turn_button = new GamepadButton(driver_gamepad, GamepadKeys.Button.A);
        dist_button = new GamepadButton(driver_gamepad, GamepadKeys.Button.B);
        time_drive_button = new GamepadButton(driver_gamepad, GamepadKeys.Button.X);


        time_drive_button.whenPressed(time_drive);

        drive_command = new DefaultDrive(drive, () -> driver_gamepad.getLeftY(), () -> driver_gamepad.getRightX());
        flywheel_joystick = new FlywheelJoystick(shooter, () -> driver_gamepad.getLeftY());
//        turn_button.whenPressed(turn_command);
//        dist_button.whenPressed(new DriveToDistance(drive, 0.5));
        shooter.setDefaultCommand(flywheel_joystick);

//        drive.setDefaultCommand(drive_command);
    }


    @Override
    public void run() {
        super.run();
        drive.addDashboardData();
        shooter.addDashboardData();
        dashboardTelemetry.update();
        telemetry.update();
    }
}