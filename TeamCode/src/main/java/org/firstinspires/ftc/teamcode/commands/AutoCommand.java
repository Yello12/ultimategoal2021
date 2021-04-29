package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.button.Button;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.utils.Units;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.function.DoubleSupplier;

@Autonomous(name = "Main Auto")
public class AutoCommand extends CommandOpMode {

    private int width = 320;
    private int height = 240;

    private RingDetector detector = new RingDetector();
    private OpenCvCamera webcam;

    private RobotHardware robot = new RobotHardware();

    private Drive drive;
    private WobbleArm wobble_arm;
    private Transport transport;
    private Shooter shooter;

    private AutoA auto_a;
    private AutoB auto_b;
    private AutoC auto_c;
    private AutoD auto_d;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private double start_timer = Double.POSITIVE_INFINITY;
    private RingDetector.RingStack last_location = RingDetector.RingStack.NONE;
    private boolean valid_height = false;
    private boolean was_valid = false;
    private boolean first_run = true;

    @Override
    public void initialize() {
        robot.init(hardwareMap);

        drive = new Drive(robot.left_drive, robot.right_drive, robot.imu);
        wobble_arm = new WobbleArm(robot.wobble_claw, robot.arm_motor);
        transport = new Transport(robot.low_belt, robot.mid_belt, robot.high_belt, robot.high_belt2);
        shooter = new Shooter(robot.back_flywheel, robot.front_flywheel, 540);
        auto_a = new AutoA(drive, wobble_arm, transport, shooter);
        auto_b = new AutoB(drive, wobble_arm, transport, shooter);
        auto_c = new AutoC(drive, wobble_arm, transport, shooter);
        auto_d = new AutoD(drive, wobble_arm, transport, shooter);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        webcam.setPipeline(detector);
        webcam.openCameraDevice();
        webcam.startStreaming(width, height, OpenCvCameraRotation.UPRIGHT);
    }


    @Override
    public void run() {
        super.run();

        if (!valid_height) {
            RingDetector.RingStack location = detector.getHeight();
            double timestamp = (double) System.nanoTime() / 1E9;
            if ((location == last_location) && !first_run) {
                if ((timestamp - start_timer) >= 4) {
                    valid_height = true;
                }
            } else {
                start_timer = (double) System.nanoTime() / 1E9;
            }
            first_run = false;
            last_location = location;
            if (location == RingDetector.RingStack.NONE) {
                telemetry.addData("Ring Height", "None");
                dashboardTelemetry.addData("Ring Height", "None");
            } else if (location == RingDetector.RingStack.ONE) {
                telemetry.addData("Ring Height", "One");
                dashboardTelemetry.addData("Ring Height", "One");

            } else if (location == RingDetector.RingStack.FOUR) {
                telemetry.addData("Ring Height", "Four");
                dashboardTelemetry.addData("Ring Height", "Four");

            }
            telemetry.addData("Timer", timestamp - start_timer);
            telemetry.addData("Valid", valid_height);
            dashboardTelemetry.addData("Timer", timestamp - start_timer);
            dashboardTelemetry.addData("Valid", valid_height);

        }
        if (was_valid != valid_height) {

            webcam.stopStreaming();
            webcam.closeCameraDevice();
            if (last_location == RingDetector.RingStack.NONE) {
                schedule(auto_a);
            } else if (last_location == RingDetector.RingStack.ONE) {
                schedule(auto_b);
            } else if (last_location == RingDetector.RingStack.FOUR) {
                schedule(auto_c);
            }

        }
        was_valid = valid_height;

//        drive.addDashboardData();
//        dashboardTelemetry.update();
        telemetry.update();
        sleep(100);
    }


}