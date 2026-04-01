package org.firstinspires.ftc.teamcode.Actions.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class MoveSpindexCommand extends CommandBase {
    public SpindexSubsystem spindex;
    public RobotHardware robot;
    public SpindexSubsystem.SpindexPoses targetPose;
    public MoveSpindexCommand(SpindexSubsystem spindex, RobotHardware robot, SpindexSubsystem.SpindexPoses spindexPose){
        this.spindex = spindex;
        this.robot = robot;
        this.targetPose = spindexPose;
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute(){
        super.execute();
        spindex.moveToPose(targetPose);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return !robot.isSpindexBusy();
    }
}
