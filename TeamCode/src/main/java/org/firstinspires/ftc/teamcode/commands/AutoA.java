package org.firstinspires.ftc.teamcode.commands;

import android.os.ParcelFileDescriptor;

import androidx.annotation.Nullable;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Shooter;
import org.firstinspires.ftc.teamcode.subsystems.Transport;
import org.firstinspires.ftc.teamcode.subsystems.WobbleArm;
import org.firstinspires.ftc.teamcode.utils.Units;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.Set;

import kotlin.Unit;

public class AutoA extends SequentialCommandGroup {


    public AutoA(Drive drive, WobbleArm wobble_arm, Transport transport, Shooter shooter) {
        addCommands(
                new AutoShoot(transport, shooter),
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Closed),
                new DriveTimed(drive, 1.0, 0),
                new TurnToAngle(drive, 146 * Units.DEGREES),
                new DriveToDistance(drive, 5.0 * Units.FEET, 0.4),
                new WobbleClawToggle(wobble_arm, WobbleArm.Mode.Open),
                new DriveToDistance(drive, -0.2 * Units.FEET, -0.2)
        );
        addRequirements(drive, wobble_arm);
    }
}