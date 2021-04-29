package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Transport extends SubsystemBase {


    public enum Mode {
        Idle,
        Index,
        Manual,

    }

    private double[] INDEX_SPEED = {0.2, 0.2, 0.2, 0.2};

    private CRServo[] motors = {null, null, null, null};

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Telemetry dashboardTelemetry = dashboard.getTelemetry();

    private Mode desired_mode;
    private double[] desired_output = {0, 0};

    public Transport(CRServo _low_belt, CRServo _mid_belt, CRServo _high_belt, CRServo _high_belt2) {
        motors[0] = _low_belt;
        motors[1] = _mid_belt;
        motors[2] = _high_belt;
        motors[3] = _high_belt2;

        desired_mode = Mode.Idle;
    }

    public void stop() {
        desired_mode = Mode.Idle;
        for (int i = 0; i < desired_output.length; i++) {
            desired_output[i] = 0;
        }

    }

    public void setStageOutput(int stage, double output) {
        desired_mode = Mode.Manual;
        desired_output[stage] = output;
    }

    public void setAllOutput(double[] output) {
        desired_mode = Mode.Idle;
        desired_output = output;
    }

    public void addDashboardData() {

    }

    @Override
    public void periodic() {
        if (desired_mode == Mode.Idle) {
            for (int i = 0; i < motors.length; i++) {
                motors[i].set(0);
            }
        } else if (desired_mode == Mode.Manual) {
            motors[0].set(desired_output[0]);
            motors[1].set(desired_output[0]);
            motors[2].set(desired_output[1]);
            motors[3].set(desired_output[1]);

        }
    }

}
