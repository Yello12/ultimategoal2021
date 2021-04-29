package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;

public class WobbleClawToggle extends CommandBase {

    private final WobbleArm wobble_arm;
    private final WobbleArm.Mode claw_mode;

    public WobbleClawToggle(WobbleArm _wobble_arm, WobbleArm.Mode mode) {
        wobble_arm = _wobble_arm;
        claw_mode = mode;
        addRequirements(wobble_arm);
    }

    @Override
    public void initialize() {
        if(claw_mode == WobbleArm.Mode.Closed){
            wobble_arm.close();
        }
        else if(claw_mode == WobbleArm.Mode.Open){
            wobble_arm.open();
        }

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}