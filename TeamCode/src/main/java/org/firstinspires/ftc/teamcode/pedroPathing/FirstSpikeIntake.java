package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

@Config
@Autonomous
public class FirstSpikeIntake extends LinearOpMode {
    public static double maxPower = 1;
    //    public static double velocityContstraint = 0.2;
//    public static double translationalContraint = 5;
    public static Pose INIT_POSE = new Pose(72, 72, Math.toRadians(180));
    public static Pose END_POSE1 = new Pose(12, 63, Math.toRadians(155));
    private PathChain moveToSpike;
    public int pathState = 0;
    public Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(INIT_POSE);
        buildPaths();
        waitForStart();
        while (opModeIsActive()){
            follower.update();
            runPaths();
        }

    }

    public void buildPaths(){
        moveToSpike = follower.pathBuilder()
                .addPath(new BezierLine(INIT_POSE, END_POSE1))
                .setLinearHeadingInterpolation(INIT_POSE.getHeading(), END_POSE1.getHeading())
                .build();
    }
    public void runPaths(){
        switch(pathState){
            case 0:
                follower.followPath(moveToSpike, maxPower, true);
                pathState += 1;
        }
    }

}
