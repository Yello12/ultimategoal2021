package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drive;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Transport;

public class Index extends CommandBase {
//    public enum State {
//        Idle,
//        Suck,
//        Spit,
//    }
//
//
//    private Intake intake;
//    private Transport transport;
//
//    public Index(Intake _intake, Transport _transport) {
//        // https://docs.google.com/spreadsheets/d/1PiBs1qji9z9AhigtD2FiE0YvE75OzcPSDtVR8vrfy8Y/edit#gid=0
//        intake = _intake;
//        transport = _transport;
//    }
//
//    @Override
//    public void initialize() {
//
//    }
//
//    @Override
//    public void execute() {
//        double speed = 0.2;
//        if (transport.hasRings(new boolean[]{false, false, false}) || transport.hasRings(new boolean[]{true, false, false}) || transport.hasRings(new boolean[]{true, true, false})) {
//            intake.suck();
//            transport.setAllOutput(new double[]{speed, speed, speed});
//        } else if (transport.hasRings(new boolean[]{false, true, false})) {
//            intake.suck();
//            transport.setAllOutput(new double[]{0, speed, speed});
//        } else if (transport.hasRings(new boolean[]{false, false, true}) || transport.hasRings(new boolean[]{true, false, true})) {
//            intake.suck();
//            transport.setAllOutput(new double[]{speed, speed, 0});
//        } else if (transport.hasRings(new boolean[]{false, true, true})) {
//            intake.suck();
//            transport.setAllOutput(new double[]{speed, 0, 0});
//        } else if (transport.hasRings(new boolean[]{true, true, true})) {
//            intake.stop();
//            transport.setAllOutput(new double[]{0, 0, 0});
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return false;
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//        transport.stop();
//        intake.stop();
//    }
}
