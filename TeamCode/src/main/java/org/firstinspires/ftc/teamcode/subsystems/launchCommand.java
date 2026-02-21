package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class launchCommand extends CommandBase {

    private launcherSubsystem launcherSubsystem;
    private RobotHardware robotHardware;
    private SpindexSubsystem spindex;
    public launchCommand(launcherSubsystem launcherSubsystem, RobotHardware robotHardware){
        this.launcherSubsystem = launcherSubsystem;
        this.robotHardware = robotHardware;
        this.spindex = new SpindexSubsystem(this.robotHardware);
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();


        launchArtifacts();

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public void launchArtifacts(){

        robotHardware.setFlywheelVelocityInTPS(launcherSubsystem.getArtifactLaunchValues().flywheelVelocity);

        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.INTAKE_POSE_3){
            robotHardware.setLaunchVisorPosition(launcherSubsystem.getArtifactLaunchValues().visorPositions.get(0));
        }

        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.LAUNCH_POSE_1){
            robotHardware.setLaunchVisorPosition(launcherSubsystem.getArtifactLaunchValues().visorPositions.get(1));
        }

        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.LAUNCH_POSE_2){
            robotHardware.setLaunchVisorPosition(launcherSubsystem.getArtifactLaunchValues().visorPositions.get(2));
        }

    }

}
