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
@Disabled
@Autonomous
public class FirstSpikeIntake extends LinearOpMode {
    public static double maxPower = 1;
    //    public static double velocityContstraint = 0.2;
//    public static double translationalContraint = 5;
    public static Pose INIT_POSE = new Pose(56, 8, Math.toRadians(180));
    public static Pose END_POSE1 = new Pose(17, 35, Math.toRadians(180));
    public static Pose END_POSE2 = new Pose(56, 12, Math.toRadians(120));
    private Pose Control_Pose = new Pose(56, 40);
    private PathChain moveToSpike;
    private PathChain moveToScore;
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
                .addPath(new BezierCurve(INIT_POSE, Control_Pose, END_POSE1))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        moveToScore = follower.pathBuilder()
                .addPath(new BezierLine(END_POSE1, END_POSE2))
                .setLinearHeadingInterpolation(END_POSE1.getHeading(), END_POSE2.getHeading())
//                .setTranslationalConstraint(translationalContraint)
//                .setVelocityConstraint(velocityContstraint)
                .build();
    }
    public void runPaths(){
        switch(pathState){
            case 0:
                follower.followPath(moveToSpike, maxPower, true);
                pathState += 1;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(moveToScore, maxPower, true);
                }
        }
    }

}
