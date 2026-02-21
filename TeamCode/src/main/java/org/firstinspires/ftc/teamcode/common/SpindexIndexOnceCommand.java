//package org.firstinspires.ftc.teamcode.common;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//
//import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
//
//public class SpindexIndexOnceCommand extends CommandBase {
//
//    private final SpindexSubsystem spindex;
//    private boolean done = false;
//
//    public SpindexIndexOnceCommand(SpindexSubsystem spindex) {
//        this.spindex = spindex;
//        addRequirements(spindex);
//    }
//
//    @Override
//    public void execute() {
//        if (!done) {
//            done = spindex.getPoseFromIndex();
//        }
//    }
//
//    @Override
//    public boolean isFinished() {
//        return done;
//    }
//}
