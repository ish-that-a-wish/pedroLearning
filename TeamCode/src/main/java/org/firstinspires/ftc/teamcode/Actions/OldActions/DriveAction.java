package org.firstinspires.ftc.teamcode.Actions.OldActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Tests.Constants;

public class DriveAction implements Action {
    private Pose initPose;
    private Pose driveToPose;
    private Follower follower;
    private PathChain path;
    public DriveAction(Pose initalizedPose, Pose endPose, HardwareMap hardwareMap){
        this.follower = Constants.createFollower(hardwareMap);
        this.initPose = initalizedPose;
        follower.setStartingPose(initPose);
        this.driveToPose = endPose;
        path = follower.pathBuilder()
                .addPath(new BezierLine(initPose, driveToPose))
                .build();
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        follower.update();
        follower.followPath(path);
        return false;
    }
}
