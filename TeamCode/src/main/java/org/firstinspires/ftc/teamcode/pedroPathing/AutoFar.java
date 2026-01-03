package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.GATE_PICKUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.GATE_PICKUP_NO_INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.HUMAN_PLAYER_PICKUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.INIT_POSE;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.SECRET_TUNNEL_AFTER_SHOOTING;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.SECRET_TUNNEL_INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.SHOOTING_POSE;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveOffLaunchLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToGate;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToGateNoIntake;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToSecretTunnelAfterShooting;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootAfterSecretTunnelShooting;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootGate;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootGateNoIntake;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootSpike1;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToShootSpike2;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToSpike1Pickup;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.moveToSpike2Pickup;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.postSecretTunnel;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.preHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.FarConstants.preSecretTunnelAfterGate;

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
    //CHANGE SO THE LIST CHECKS ANY ACTION HAS BEEN CLICKED TWICE FOR EVERYTHING BESIDES SECRET TUNNEL AND HUMAN PLAYER
    public boolean isFirstSpikePressed = false;
    public boolean isSecondSpikePressed = false;
    public boolean isGatePressed = false;
    public boolean isMovedOffLineAtEnd = false;
    /* ---------------- ENUM ---------------- */

    public enum AutoAction {
        FIRST_SPIKE,
        SECOND_SPIKE,
        GATE_PICKUP,
        HUMAN_PLAYER,
        GATE_NO_INTAKE,
        MOVE_OFF_LINE,
        SECRET_TUNNEL
    }
    /* ---------------- FIELDS ---------------- */

    private ArrayList<AutoAction> orderToPickup = new ArrayList<>();
    private int actionIndex = 0;

    private Follower follower;
    private final ElapsedTime waitTimer = new ElapsedTime();

    private double waitSeconds = 5.0;

    /* ---------------- PATHS ---------------- */

    private PathChain pickUpSpike1;
    private PathChain pickUpSpike2;
    private PathChain gatePickup;
    private PathChain humanPlayerPickup;
    public PathChain gateNoIntake;
    public PathChain secretTunnelAfterGate;
    public PathChain gateNoShoot;
    public PathChain secretTunnelAfterShot;
    public Boolean waiting = false;
    public ArrayList<Path> pathsToAddWait = new ArrayList<>();
    /* ---------------- OPMODE ---------------- */


    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        buildPaths();
        /* -------- INIT BUTTON SELECTION -------- */

        while (opModeInInit()) {
            if(gamepad1.dpadUpWasReleased()) {
                if (!orderToPickup.isEmpty()) {
                    //wait logic
                    AutoAction lastAction = orderToPickup.get(orderToPickup.size() - 1); // get the last element of the order to pickup
                    if(lastAction == AutoAction.GATE_NO_INTAKE){
                        telemetry.addData("ADDING WAITS TO: ", "FIRST SPIKE");
                        Log.i("WAITING: ", "FIRST SPIKE");
                        pathsToAddWait.add(moveToGateNoIntake);
                        pathsToAddWait.add(moveToShootGateNoIntake);
                        gateNoIntake = buildPathChainWithWait(pathsToAddWait, Math.toRadians(270), 0.9);
                        pathsToAddWait.clear();
                    }
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
            if (gamepad1.aWasReleased() && !isFirstSpikePressed) {
                orderToPickup.add(AutoAction.FIRST_SPIKE);
                isFirstSpikePressed = true;
            }

            if (gamepad1.bWasReleased() && !isSecondSpikePressed) {
                orderToPickup.add(AutoAction.SECOND_SPIKE);
                isSecondSpikePressed = true;
            }

            if (gamepad1.yWasReleased() && !isGatePressed) {
                orderToPickup.add(AutoAction.GATE_PICKUP);
                isGatePressed = true;
            }

            if (gamepad1.xWasReleased()) {
                orderToPickup.add(AutoAction.HUMAN_PLAYER);
            }
            if(gamepad1.dpadDownWasReleased()){
                orderToPickup.add(AutoAction.GATE_NO_INTAKE);
            }
            if(gamepad1.dpadLeftWasReleased()){
                orderToPickup.add(AutoAction.MOVE_OFF_LINE);
            }
            if(gamepad1.dpadRightWasReleased()){
                orderToPickup.add(AutoAction.SECRET_TUNNEL);
            }
            if (orderToPickup.contains(AutoAction.FIRST_SPIKE) && orderToPickup.contains(AutoAction.HUMAN_PLAYER)) {
                if (orderToPickup.indexOf(AutoAction.FIRST_SPIKE) > orderToPickup.indexOf(AutoAction.HUMAN_PLAYER)) {
                    telemetry.addData("WARNING: ", "INTAKING FIRST SPIKE BEFORE HUMAN PLAYER");
                }
            }
            if (orderToPickup.contains(AutoAction.SECOND_SPIKE) && orderToPickup.contains(AutoAction.GATE_NO_INTAKE)) {
                if (orderToPickup.indexOf(AutoAction.SECOND_SPIKE) > orderToPickup.indexOf(AutoAction.GATE_NO_INTAKE)) {
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
            if(!isMovedOffLineAtEnd){
                moveMoveOffLineToEnd();
                isMovedOffLineAtEnd = true;
            }
            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
                follower.breakFollowing();
            }
            follower.update();
            runPaths();
            telemetry.addData("ACTION ORDER: ", orderToPickup);
            telemetry.addData("Current Action Index", actionIndex);
            telemetry.addData("Queue Size", orderToPickup.size());
            telemetry.update();
        }
    }

    /* ---------------- PATH BUILDING ---------------- */

    private void buildPaths() {

        pickUpSpike1 = buildAPath(moveToSpike1Pickup, moveToShootSpike1, Math.toRadians(180));

        pickUpSpike2 = buildAPath(moveToSpike2Pickup, moveToShootSpike2, Math.toRadians(180));

        gatePickup = buildAPath(moveToGate, moveToShootGate, GATE_PICKUP.getHeading(), SHOOTING_POSE.getHeading());

        humanPlayerPickup = follower.pathBuilder()
                .addPath(preHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToShootHumanPlayer)
                .setLinearHeadingInterpolation(HUMAN_PLAYER_PICKUP.getHeading(), SHOOTING_POSE.getHeading())
                .build();

        gateNoIntake = buildAPath(moveToGate, moveToShootGateNoIntake, Math.toRadians(180));

        secretTunnelAfterGate = follower.pathBuilder()
                .addPath(preSecretTunnelAfterGate)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(postSecretTunnel)
                .setLinearHeadingInterpolation(SECRET_TUNNEL_INTAKE.getHeading(), SHOOTING_POSE.getHeading())
                .build();
        gateNoShoot = follower.pathBuilder()
                .addPath(moveToGate)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), GATE_PICKUP_NO_INTAKE.getHeading())
                .build();
        secretTunnelAfterShot = buildAPath(moveToSecretTunnelAfterShooting, moveToShootAfterSecretTunnelShooting, SECRET_TUNNEL_AFTER_SHOOTING.getHeading(), SHOOTING_POSE.getHeading());
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
//        AutoAction currentAction = AutoAction.MOVE_OFF_LINE;
        AutoAction currentAction = orderToPickup.get(actionIndex);

        switch (currentAction) {
            case MOVE_OFF_LINE:
                follower.followPath(moveOffLaunchLine);
                break;
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
            case GATE_NO_INTAKE:
                if(isSecretTunnelAfterGate()){
                    follower.followPath(moveToGateNoIntake);
                }
                if(!isSecretTunnelAfterGate()) {
                    follower.followPath(gateNoIntake);
                }
                break;
            case SECRET_TUNNEL:
                if(isSecretTunnelAfterGate()) {
                    Log.i("SECRET TUNNEL POSE: ", "AFTER GATE");
                    follower.followPath(secretTunnelAfterGate);
                }
                if(!isSecretTunnelAfterGate()){
                    Log.i("SECRET TUNNEL POSE: ", "NOT AFTER GATE");
                    follower.followPath(secretTunnelAfterShot);
                }
        }
            actionIndex++; // Advance to next queued action
    }
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
    public void moveMoveOffLineToEnd(){
        if(orderToPickup.contains(AutoAction.MOVE_OFF_LINE)) {
            int index = orderToPickup.indexOf(AutoAction.MOVE_OFF_LINE);
            orderToPickup.remove(index);
            orderToPickup.add(orderToPickup.size(), AutoAction.MOVE_OFF_LINE);
        }
        else{
        }
    }
    public boolean isSecretTunnelAfterGate(){
        if(orderToPickup.contains(AutoAction.GATE_NO_INTAKE) && orderToPickup.contains(AutoAction.SECRET_TUNNEL)) {
            int gateIndexNoIntake = orderToPickup.indexOf(AutoAction.GATE_NO_INTAKE);
            int indexOfSecretTunnel = orderToPickup.indexOf(AutoAction.SECRET_TUNNEL);
            Log.i("GATE INDEX NO INTAKE: ", String.valueOf(gateIndexNoIntake));
            Log.i("SECRET TUNNEL INDEX: ", String.valueOf(indexOfSecretTunnel));
            return indexOfSecretTunnel - 1 == gateIndexNoIntake;
        }
        return false;
    }
    public PathChain buildAPath(Path moveToPath, Path shootAtPath, double radHeading){
        PathChain path1 = follower.pathBuilder()
                .addPath(moveToPath)
                .setConstantHeadingInterpolation(radHeading)
                .addPath(shootAtPath)
                .setConstantHeadingInterpolation(radHeading)
                .build();
        return path1;
    }
    public PathChain buildAPath(Path moveToPath, Path shootAtPath, double pickupHeading, double shootingHeading){
        PathChain path1 = follower.pathBuilder()
                .addPath(moveToPath)
                .setLinearHeadingInterpolation(shootingHeading, pickupHeading)
                .addPath(shootAtPath)
                .setLinearHeadingInterpolation(pickupHeading, shootingHeading)
                .build();
        return path1;
    }
}
