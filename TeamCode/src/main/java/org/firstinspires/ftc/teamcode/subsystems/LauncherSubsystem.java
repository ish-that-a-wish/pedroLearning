package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.LimelightYDT;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class LauncherSubsystem extends SubsystemBase {

    public RobotHardware robotHardware;


    public LimelightYDT limelightYDT;

    LimelightAprilTagHelper limelightAprilTagHelper;

    public LauncherSubsystem(RobotHardware robotHardware, LimelightAprilTagHelper limelightAprilTagHelper){
        this.robotHardware = robotHardware;
        this.limelightAprilTagHelper = limelightAprilTagHelper;
//      ?""""
    }




    public BallLaunchParameters getArtifactLaunchValues(){
        limelightYDT = limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
        if(limelightYDT == null){
            Log.i("== ROBOT HARDWARE ==", "null");
            return LaunchParametersLookup.getBallLaunchParameters(90);
        }
        return LaunchParametersLookup.getBallLaunchParameters(limelightYDT.distance);
    }




}
