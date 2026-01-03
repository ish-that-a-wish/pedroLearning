package org.firstinspires.ftc.teamcode.pedroPathing.Pedro;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner.MoveSlideAction;

@Config
@Disabled
@Autonomous
public class FirstSpike extends LinearOpMode {
    public static double maxPower = 0.85;
//    public static double velocityContstraint = 0.2;
//    public static double translationalContraint = 5;
    public static Pose INIT_POSE = new Pose(56, 8, Math.toRadians(180));
    public static Pose FIRST_SPIKE = new Pose(8, 10, Math.toRadians(180));
    public static Pose SECOND_SPIKE = new Pose(27, 35, Math.toRadians(180));
    public static Pose THIRD_SPIKE = new Pose(27, 60, Math.toRadians(180));
    public static Pose GATE_POSE = new Pose(12, 70, Math.toRadians(90));
    public static Pose SHOOTING_POSE = new Pose(56, 18, Math.toRadians(180));
    private Pose CONTROL_POINT_FIRST_SPIKE = new Pose(56, 40);
    private Pose CONTROL_POINT_SECOND_SPIKE = new Pose(55, 63);
    private Pose CONTROL_GATE = new Pose(26, 69);
    private PathChain moveToFirstSpike, moveToSecondSpike, moveToThirdSpike, moveToGate, shootThirdSpike;
    public int pathState = 0;
    public Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(INIT_POSE);
        buildPaths();
        waitForStart();
        while (opModeIsActive()){
            telemetry.addData("On Path state: ", pathState);
            telemetry.addData("Current Bot Pose: ", follower.getPose());
            telemetry.update();
            follower.update();
            runPaths();
        }

    }

    public void buildPaths(){
        moveToFirstSpike = follower.pathBuilder()
                .addPath(new BezierLine(INIT_POSE, FIRST_SPIKE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 500, false)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(FIRST_SPIKE, SHOOTING_POSE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 0, true)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        moveToSecondSpike = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_FIRST_SPIKE, SECOND_SPIKE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 500, false)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(SECOND_SPIKE, SHOOTING_POSE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 0, true)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        moveToThirdSpike = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, THIRD_SPIKE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 500, false)))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();
        moveToGate = follower.pathBuilder()
                .addPath(new BezierCurve(THIRD_SPIKE, CONTROL_GATE, GATE_POSE))
                .setLinearHeadingInterpolation(THIRD_SPIKE.getHeading(), GATE_POSE.getHeading())
                .build();
        shootThirdSpike = follower.pathBuilder()
                .addPath(new BezierLine(GATE_POSE, SHOOTING_POSE))
                .addParametricCallback(0.2, ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 0, true)))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
//        moveToSpike = follower.pathBuilder()
//                .addPath(new BezierCurve(INIT_POSE, Control_Pose, END_POSE1))
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();
//        moveToScore = follower.pathBuilder()
//                .addPath(new BezierLine(END_POSE1, END_POSE2))
//                .setLinearHeadingInterpolation(END_POSE1.getHeading(), END_POSE2.getHeading())
////                .setTranslationalConstraint(translationalContraint)
////                .setVelocityConstraint(velocityContstraint)
//                .build();
    }
    public void runPaths(){
        switch(pathState){
            case 0:
                Log.i("MOVING TO FIRST SPIKE", "PATHSTATE: " + pathState);
                follower.followPath(moveToFirstSpike, maxPower, true);
                pathState += 1;
            case 1:
                if(!follower.isBusy()){
                    Log.i("MOVING TO SECOND SPIKE", "PATHSTATE: " + pathState);
                    follower.followPath(moveToSecondSpike, maxPower, true);
                    pathState+=1;
                    break;
                }
            case 2:
                Log.i("MOBING TO THIRD SPIKE","PATHSTATE: " + pathState);
                if(!follower.isBusy()){
                    follower.followPath(moveToThirdSpike, maxPower, true);
                    pathState += 1;
                    break;
                }
            case 3:
                Log.i("MOVING TO GATE","PATHSTATE: " + pathState);
                if(!follower.isBusy()){
                    follower.followPath(moveToGate, maxPower, true);
                    pathState += 1;
                    break;
                }
            case 4:
                Log.i("SHOOTING THIRD SPIKE","PATHSTATE: " + pathState);
                if(!follower.isBusy()){
                    follower.followPath(shootThirdSpike, maxPower, true);
                    pathState += 1;
                    break;
                }
        }
    }

}
