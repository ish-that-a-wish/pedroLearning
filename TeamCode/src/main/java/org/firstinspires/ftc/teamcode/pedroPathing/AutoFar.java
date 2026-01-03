package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Curve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.CallBacks.WaitCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;
import com.acmerobotics.roadrunner.Action.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.Collection;

@Autonomous
public class AutoFar extends LinearOpMode {

    /* ---------------- ENUM ---------------- */

    public enum AutoAction {
        FIRST_SPIKE,
        SECOND_SPIKE,
        GATE_PICKUP,
        HUMAN_PLAYER
    }
    /* ---------------- FIELDS ---------------- */

    private ArrayList<AutoAction> orderToPickup = new ArrayList<>();
    private int actionIndex = 0;

    private Follower follower;
    private final ElapsedTime waitTimer = new ElapsedTime();

    private double waitSeconds = 5.0;


    /* ---------------- POSES ---------------- */
    public Pose INIT_POSE = new Pose(56, 8, Math.toRadians(180));

    public static Pose FIRST_SPIKE = new Pose(27, 35, Math.toRadians(180));
    public static Pose SECOND_SPIKE = new Pose(27, 60, Math.toRadians(180));

    public static Pose SHOOTING_POSE = new Pose(56, 18, Math.toRadians(180));
    public static Pose GATE_PICKUP = new Pose(14, 63, Math.toRadians(135));
    public static Pose SHOOTING_AFTER_GATE = new Pose(56, 18, Math.toRadians(270));

    public static Pose PRE_HUMAN_PLAYER = new Pose(15, 30, Math.toRadians(270));
    public static Pose HUMAN_PLAYER_PICKUP = new Pose(15, 20, Math.toRadians(270));

    private Pose CONTROL_POINT_FIRST_SPIKE = new Pose(56, 40);
    private Pose CONTROL_POINT_SECOND_SPIKE = new Pose(55, 63);

    /* ---------------- PATHS ---------------- */

    private PathChain pickUpSpike1;
    private PathChain pickUpSpike2;
    private PathChain gatePickup;
    private PathChain humanPlayerPickup;
    public Boolean waiting = false;
    public ArrayList<Path> pathsToAddWait = new ArrayList<>();
    /* ---------------- OPMODE ---------------- */

    public Path moveToSpike1Pickup = new Path(new BezierCurve(INIT_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE));
    public Path moveToShootSpike1 = new Path(new BezierLine(FIRST_SPIKE, SHOOTING_POSE));
    public Path moveToSpike2Pickup = new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE));
    public Path moveToShootSpike2 = new Path(new BezierLine(SECOND_SPIKE, SHOOTING_POSE));
    public Path moveToGate = new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP));
    public Path moveToShootGate = new Path(new BezierLine(GATE_PICKUP, SHOOTING_AFTER_GATE));
    public Path preHumanPlayer = new Path(new BezierLine(SHOOTING_AFTER_GATE, PRE_HUMAN_PLAYER));
    public Path moveToHumanPlayer = new Path(new BezierLine(PRE_HUMAN_PLAYER, HUMAN_PLAYER_PICKUP));
    public Path moveToShootHumanPlayer = new Path(new BezierLine(HUMAN_PLAYER_PICKUP, SHOOTING_AFTER_GATE));

    @Override
    public void runOpMode() throws InterruptedException {
//        pathsToAddWait.add(
//                new Path(new BezierCurve(INIT_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE))
//        );
//        pathsToAddWait.add(
//                new Path(new BezierLine(FIRST_SPIKE, SHOOTING_POSE))
//        );
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        buildPaths();
//        pickUpSpike1 = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 0.9);
        /* -------- INIT BUTTON SELECTION -------- */

        while (opModeInInit()) {
            if(gamepad1.dpadUpWasReleased()) {
                if (!orderToPickup.isEmpty()) {
                    //wait logic
                    AutoAction lastAction = orderToPickup.get(orderToPickup.size() - 1); // get the last element of the order to pickup
                    if (lastAction == AutoAction.FIRST_SPIKE) {
                        telemetry.addData("ADDING WAITS TO: ", "FIRST SPIKE");
                        Log.i("WAITING: ", "FIRST SPIKE");
                        pathsToAddWait.add(moveToSpike1Pickup);
                        pathsToAddWait.add(moveToShootSpike1);
                        pickUpSpike1 = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 0.9);
                        pathsToAddWait.clear();
                    }
                    if (lastAction == AutoAction.SECOND_SPIKE) {
                        telemetry.addData("ADDING WAITS TO: ", "SECOND SPIKE");
                        Log.i("WAITING: ", "SECOND SPIKE");
                        pathsToAddWait.add(moveToSpike2Pickup);
                        pathsToAddWait.add(moveToShootSpike2);
                        pickUpSpike2 = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 0.9);
                        pathsToAddWait.clear();
                    }
                    if (lastAction == AutoAction.GATE_PICKUP) {
                        telemetry.addData("ADDING WAITS TO: ", "GATE PICKUP");
                        Log.i("WAITING: ", "GATE PICKUP");
                        pathsToAddWait.add(moveToGate);
                        pathsToAddWait.add(moveToShootGate);
                        gatePickup = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 0.9);
                        pathsToAddWait.clear();
                    }
                    if (lastAction == AutoAction.HUMAN_PLAYER) {
                        telemetry.addData("ADDING WAITS TO: ", "HUMAN PLAYER");
                        Log.i("WAITING: ", "HUMAN PLAYER");
                        pathsToAddWait.add(preHumanPlayer);
                        pathsToAddWait.add(moveToHumanPlayer);
                        pathsToAddWait.add(moveToShootHumanPlayer);
                        humanPlayerPickup = buildPathChainWithWait(pathsToAddWait, Math.toRadians(270), 0.9);
                        pathsToAddWait.clear();
                    }

                }
            }
            if (gamepad1.aWasReleased()) {
                orderToPickup.add(AutoAction.FIRST_SPIKE);
            }

            if (gamepad1.bWasReleased()) {
                orderToPickup.add(AutoAction.SECOND_SPIKE);
            }

            if (gamepad1.yWasReleased()) {
                orderToPickup.add(AutoAction.GATE_PICKUP);
            }

            if (gamepad1.xWasReleased()) {
                orderToPickup.add(AutoAction.HUMAN_PLAYER);
            }
            if (orderToPickup.contains(AutoAction.FIRST_SPIKE) && orderToPickup.contains(AutoAction.HUMAN_PLAYER)) {
                if (orderToPickup.indexOf(AutoAction.FIRST_SPIKE) > orderToPickup.indexOf(AutoAction.HUMAN_PLAYER)) {
                    telemetry.addData("WARNING: ", "INTAKING FIRST SPIKE BEFORE HUMAN PLAYER");
                }
            }
            if (orderToPickup.contains(AutoAction.SECOND_SPIKE) && orderToPickup.contains(AutoAction.GATE_PICKUP)) {
                if (orderToPickup.indexOf(AutoAction.SECOND_SPIKE) > orderToPickup.indexOf(AutoAction.GATE_PICKUP)) {
                    telemetry.addData("WARNING: ", "INTAKING SECOND SPIKE BEFORE GATE");
                }
            }
            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderToPickup.toString());
            telemetry.update();
        }

        waitForStart();

        /* -------- AUTON LOOP -------- */

        while (opModeIsActive()) {

            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
                follower.breakFollowing();
            }
            follower.update();
            runPaths();

            telemetry.addData("Current Action Index", actionIndex);
            telemetry.addData("Queue Size", orderToPickup.size());
            telemetry.update();
        }
    }

    /* ---------------- PATH BUILDING ---------------- */

    private void buildPaths() {

        pickUpSpike1 = follower.pathBuilder()
                .addPath(moveToSpike1Pickup)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(moveToShootSpike1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickUpSpike2 = follower.pathBuilder()
                .addPath(moveToSpike2Pickup)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(moveToShootSpike2)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        gatePickup = follower.pathBuilder()
                .addPath(moveToGate)
                .setLinearHeadingInterpolation(
                        SHOOTING_POSE.getHeading(),
                        GATE_PICKUP.getHeading()
                )
                .addPath(moveToShootGate)
                .setLinearHeadingInterpolation(
                        GATE_PICKUP.getHeading(),
                        SHOOTING_AFTER_GATE.getHeading()
                )
                .build();

        humanPlayerPickup = follower.pathBuilder()
                .addPath(preHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToShootHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();
    }

    /* ---------------- STATE MACHINE ---------------- */

    private void runPaths() {

        // No more actions to run
        if (actionIndex >= orderToPickup.size()) {
            return;
        }

        // Wait until the current path finishes
        if (follower.isBusy()) {
            return;
        }
        if(waiting){
            return;
        }
        AutoAction currentAction = orderToPickup.get(actionIndex);

        switch (currentAction) {

            case FIRST_SPIKE:
                follower.followPath(pickUpSpike1);
                break;

            case SECOND_SPIKE:
                follower.followPath(pickUpSpike2);
                break;

            case GATE_PICKUP:
                follower.followPath(gatePickup);
                break;

            case HUMAN_PLAYER:
                follower.followPath(humanPlayerPickup);
                break;
        }
            actionIndex++; // Advance to next queued action
    }

//    public void addWaitToPath(ArrayList<PathChain> paths, double whenToPause, int numOfPaths) {
//        Log.i("IN FUNCTION: ", "ADD WAIT TO PATH");
//        Log.i("ADDING WAIT TO PATHS: ", paths.toString());
//        if (pathIndex >= paths.size()) {
//            Log.i("PATH INDEX GREATER THAN SIZE OF LIST: ", "EXITING");
//            return;
//        }
//        ArrayList<PathChain> PathsWithWait = new ArrayList<>();
//        for (PathChain path : paths) {
//            int pathToAddIndex = 0;
//            Log.i("GOT LENGTH OF PATH: ", path.toString());
//            Log.i("GOT NUM OF PATHS: ", String.valueOf(numOfPaths));
//            while (pathToAddIndex <= numOfPaths) {
//                Log.i("ADDING PATHS TO INDEX: ", String.valueOf(pathToAddIndex));
//                PathChain newPath = follower.pathBuilder()
//                        .addPath(path.getPath(pathToAddIndex))
//                        .addParametricCallback(whenToPause,
//                                this::pauseAndWait)
//                        .build();
//                PathsWithWait.add(newPath);
//                Log.i("ADDING NEW PATH TO LIST", "NEW PATH");
//                pathToAddIndex += 1;
//            }
//        }
//    }

    public PathChain buildPathChainWithWait(
            ArrayList<Path> paths,
            double headingRadians,
            double whenToPause
    ) {
        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path list is null or empty");
        }

        PathBuilder builder = follower.pathBuilder();

        for (Path path : paths) {
            builder.addPath(path)
                    .setConstantHeadingInterpolation(headingRadians);
        }

        builder.addParametricCallback(
                whenToPause,
                this::pauseAndWait
        );

        return builder.build();
    }


    public void pauseAndWait() {
        Log.i("IN FUNCTION: ", "PAUSE AND WAIT");
        follower.pausePathFollowing();
        waitTimer.reset();
        waiting = true;
        Log.i("EXITING FUNCTION: ", "PAUSE AND WAIT");
    }
}
