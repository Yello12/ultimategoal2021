package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.opencv.core.Scalar;

@Config
public class Constants {
    public static double TURN_KP = 1; //1.7, 0.02, 0.1, 0
    public static double TURN_KI = 0.1;
    public static double TURN_KD = 0;
    public static double TURN_KF = 0;

    public static double BACK_MOTOR_KP = 0;
    public static double BACK_MOTOR_KI = 0;
    public static double BACK_MOTOR_KD = 0;
    public static double BACK_MOTOR_KF = 0;

    public static double BACK_MOTOR_KS = 0;
    public static double BACK_MOTOR_KV = 0;

    public static double CAM_LOW_H = 5;
    public static double CAM_LOW_S = 50;
    public static double CAM_LOW_V = 70;

    public static double CAM_HIGH_H = 35;
    public static double CAM_HIGH_S = 255;
    public static double CAM_HIGH_V = 255;
    // other constants
}
