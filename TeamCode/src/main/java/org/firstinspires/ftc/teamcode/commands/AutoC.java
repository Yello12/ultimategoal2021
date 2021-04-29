package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.utils.Units;

public class AutoC extends SequentialCommandGroup {


    public AutoC(Drive drive, WobbleArm wobble_arm, Transport transport, Shooter shooter) {
        addCommands(
                new AutoShoot(transport, shooter),
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Closed),
                new DriveTimed(drive, 1.0, 0),

                new TurnToAngle(drive, 155* Units.DEGREES),

                new DriveToDistance(drive, 8.2 * Units.FEET, 0.4),
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Open),
                new DriveToDistance(drive, -2.5 * Units.FEET, -0.3)

        );
        addRequirements(drive, wobble_arm);
    }
}