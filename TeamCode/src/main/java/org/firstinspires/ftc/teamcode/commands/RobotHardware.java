/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.commands.DefaultDrive;
import org.firstinspires.ftc.teamcode.commands.DriveTimed;
import org.firstinspires.ftc.teamcode.commands.FlywheelJoystick;
import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

public class RobotHardware {
    public Motor left_drive, right_drive, back_flywheel, front_flywheel, arm_motor, intake_roller;
    public ServoEx wobble_claw;
    public CRServo intake_belt, low_belt, mid_belt, high_belt, high_belt2;
    public GyroEx imu;

    HardwareMap hardware_map = null;

    public double WOBBLE_CLAW_MIN = Math.toRadians(0);
    public double WOBBLE_CLAW_MAX = Math.toRadians(270);

    public RobotHardware() {

    }

    public void init(HardwareMap _hardware_map) {
        hardware_map = _hardware_map;

        left_drive = new Motor(hardware_map, "left_drive");
        right_drive = new Motor(hardware_map, "right_drive");
        left_drive.setInverted(true);
        back_flywheel = new Motor(hardware_map, "back_flywheel");
        front_flywheel = new Motor(hardware_map, "front_flywheel");
        back_flywheel.setInverted(true);
        front_flywheel.setInverted(true);
        arm_motor = new Motor(hardware_map, "arm_motor");
        arm_motor.setInverted(true);

        wobble_claw = new SimpleServo(hardware_map, "wobble_claw", Math.toDegrees(WOBBLE_CLAW_MAX), Math.toDegrees(WOBBLE_CLAW_MIN));

        intake_roller = new Motor(hardware_map, "intake_roller");
        intake_belt = new CRServo(hardware_map, "intake_belt");
        intake_belt.setInverted(false);

        low_belt = new CRServo(hardware_map, "low_belt");
        mid_belt = new CRServo(hardware_map, "mid_belt");
        high_belt = new CRServo(hardware_map, "high_belt");
        high_belt2 = new CRServo(hardware_map, "high_belt2");
        low_belt.setInverted(true);
        mid_belt.setInverted(false);
        high_belt.setInverted(true);
        high_belt2.setInverted(true);

        imu = new RevIMU(hardware_map, "imu");
        imu.init();

        left_drive.set(0);
        right_drive.set(0);
//        back_flywheel.set(0);
//        front_flywheel.set(0);
    }
}

