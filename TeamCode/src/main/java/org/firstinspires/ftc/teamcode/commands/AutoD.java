package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.utils.Units;

public class AutoD extends SequentialCommandGroup {


    public AutoD(Drive drive, WobbleArm wobble_arm, Transport transport, Shooter shooter) {
        addCommands(
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Open),
                new PulseArm(wobble_arm,0.5,0.2),
                new DriveToDistance(drive, 1.9 * Units.FEET, 0.2),
                new TurnToAngle(drive, 20 * Units.DEGREES),
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Closed)
//                new DriveToDistance(drive, 1.0 * Units.FEET, 0.2),
//                new DriveTimed(drive, 2, 0),
//                new DriveToDistance(drive, -1.8 * Units.FEET, -0.2)
//                new TurnToAngle(drive, -90 * Units.DEGREES),
//                new TurnToAngle(drive, -179 * Units.DEGREES)
//                new DriveToDistance(drive, 1.0 * Units.FEET, 0.2)

        );
        addRequirements(drive, wobble_arm);
    }
}