package org.firstinspires.ftc.teamcode.Actions.Commands;


import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Config
public class LaunchCommand extends CommandBase {
    public static double FLYWHEEL_PERCENT = 0.55;

    public static double firstVisor = 0.67;

    public static double secondVisor = 0.64;

    public static double thirdVisor = 0.64;


    private LauncherSubsystem launcherSubsystem;
    private RobotHardware robotHardware;
    private SpindexSubsystem spindex;
    public LaunchCommand(LauncherSubsystem launcherSubsystem, RobotHardware robotHardware){
        this.launcherSubsystem = launcherSubsystem;
        this.robotHardware = robotHardware;
        this.spindex = new SpindexSubsystem(this.robotHardware);
    }

    @Override
    public void initialize() {
        robotHardware.setLimelightPipeline(6);
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

        robotHardware.setFlywheelVelocityInTPS(FLYWHEEL_PERCENT * 2800);
//        if(launcherSubsystem.limelightYDT != null) {
//            Log.i("Launcher", "Distance: " + launcherSubsystem.limelightYDT.distance);
//        }
//        if(robotHardware.getLatestLimelightResults().isValid()){
//            Log.i("== ROBOT HARDWARE ==", "Limelight results are valid");
//        }
//
//        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.INTAKE_POSE_3){
//            robotHardware.setLaunchVisorPosition(firstVisor);
//        }
//
//        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.LAUNCH_POSE_1){
//            robotHardware.setLaunchVisorPosition(secondVisor);
//        }
//
//        if(spindex.getCurrentPose() == SpindexSubsystem.SpindexPoses.LAUNCH_POSE_2){
//            robotHardware.setLaunchVisorPosition(thirdVisor);
//        }

    }

}
