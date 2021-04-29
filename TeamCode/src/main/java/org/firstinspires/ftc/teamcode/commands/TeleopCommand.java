package org.firstinspires.ftc.teamcode.commands;

import android.graphics.ColorSpace;
import android.telephony.AccessNetworkConstants;

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
import org.firstinspires.ftc.teamcode.commands.ArmJoystick;
import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.FlywheelJoystick;
import org.firstinspires.ftc.teamcode.commands.ManualIndex;
import org.firstinspires.ftc.teamcode.commands.RobotHardware;
import org.firstinspires.ftc.teamcode.commands.WobbleClawToggle;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.utils.Units;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

@TeleOp(name = "Main TeleOp")
public class TeleopCommand extends CommandOpMode {


    static final double SHOOTER_CPR = 28;

    private RobotHardware robot = new RobotHardware();

    private Drive drive;
    private Intake intake;
    private WobbleArm wobble_arm;
    private Transport transport;
    private Shooter shooter;

    private GamepadEx driver_gamepad;
    private GamepadEx operator_gamepad;

    private Button wobble_close_button;
    private Button wobble_open_button;

    private DefaultDrive drive_command;
    private WobbleClawToggle wobble_close;
    private WobbleClawToggle wobble_open;
    private ArmJoystick arm_joystick;
    private ManualIndex manual_index;
    private FlywheelJoystick flywheel_joystick;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void initialize() {
        robot.init(hardwareMap);

        drive = new Drive(robot.left_drive, robot.right_drive, robot.imu);
        wobble_arm = new WobbleArm(robot.wobble_claw, robot.arm_motor);
        intake = new Intake(robot.intake_roller, robot.intake_belt);
        transport = new Transport(robot.low_belt, robot.mid_belt, robot.high_belt, robot.high_belt2);
        shooter = new Shooter(robot.back_flywheel, robot.front_flywheel, 28);
        driver_gamepad = new GamepadEx(gamepad1);
        operator_gamepad = new GamepadEx(gamepad2);
        DoubleSupplier left_y = () -> -operator_gamepad.getLeftY();
        DoubleSupplier right_y = () -> -operator_gamepad.getRightY();

        wobble_close_button = new GamepadButton(operator_gamepad, GamepadKeys.Button.A);
        wobble_open_button = new GamepadButton(operator_gamepad, GamepadKeys.Button.B);
        wobble_close = new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Closed);
        wobble_open = new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Open);
        arm_joystick = new ArmJoystick(wobble_arm, left_y);
        BooleanSupplier left_bumper = () -> operator_gamepad.getButton((GamepadKeys.Button.LEFT_BUMPER));
        BooleanSupplier right_bumper = () -> operator_gamepad.getButton((GamepadKeys.Button.RIGHT_BUMPER));
//        BooleanSupplier x_button = () -> operator_gamepad.getButton((GamepadKeys.Button.));
        BooleanSupplier shooter_button = () -> (operator_gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.5);
        BooleanSupplier shooter_reverse = () -> (operator_gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.5);

        flywheel_joystick = new FlywheelJoystick(transport, shooter, left_y, right_y, shooter_button, shooter_reverse);

        manual_index = new ManualIndex(intake, transport, left_y, right_y, left_bumper, right_bumper);

        wobble_close_button.whenPressed(wobble_close);
        wobble_open_button.whenPressed(wobble_open);

        drive_command = new DefaultDrive(drive, driver_gamepad);
        intake.setDefaultCommand(manual_index);
        drive.setDefaultCommand(drive_command);
        shooter.setDefaultCommand(flywheel_joystick);
        wobble_arm.setDefaultCommand(arm_joystick);
    }


    @Override
    public void run() {
        super.run();
        drive.addDashboardData();
        dashboardTelemetry.update();
        telemetry.addData("Dist", drive.getAverageEncoderDistance());
//        telemetry.addData("Counts",wobble_arm.getCounts());
        telemetry.update();
    }
}