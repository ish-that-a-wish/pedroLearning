package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.LimelightYDT;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class launcherSubsystem extends SubsystemBase {

    public RobotHardware robotHardware;


    public LimelightYDT limelightYDT;

    LimelightAprilTagHelper limelightAprilTagHelper;

    public launcherSubsystem(RobotHardware robotHardware, LimelightAprilTagHelper limelightAprilTagHelper) {
        this.robotHardware = robotHardware;
        this.limelightAprilTagHelper = limelightAprilTagHelper;
//      ?""""
    }


    public LimelightYDT getLimelightLaunchValues() {
        return limelightAprilTagHelper.getGoalYawDistanceToleranceFromCurrentPosition();
    }

    public BallLaunchParameters getArtifactLaunchValues() {
        limelightYDT = getLimelightLaunchValues();
        if (limelightYDT == null) {
            return LaunchParametersLookup.getBallLaunchParameters(67.0);
        }
        return LaunchParametersLookup.getBallLaunchParameters(limelightYDT.distance);
    }
}

