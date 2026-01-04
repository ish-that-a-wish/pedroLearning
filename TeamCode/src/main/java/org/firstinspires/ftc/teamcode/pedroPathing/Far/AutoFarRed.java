package org.firstinspires.ftc.teamcode.pedroPathing.Far;

import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.GATE_PICKUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.GATE_PICKUP_NO_INTAKE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.HUMAN_PLAYER_PICKUP;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.INIT_POSE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.MOVE_OFF_LINE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.SECRET_TUNNEL_AFTER_SHOOTING;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.SHOOTING_POSE;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveOffLaunchLine;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToGate;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToGateNoIntake;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToSecretTunnelAfterShooting;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootAfterSecretTunnelShooting;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootGate;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootGateNoIntake;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootSpike1;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToShootSpike2;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToSpike1Pickup;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.moveToSpike2Pickup;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.postSecretTunnel;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.preHumanPlayer;
import static org.firstinspires.ftc.teamcode.pedroPathing.Far.RedFarConstants.preSecretTunnelAfterGate;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

@Autonomous
public class AutoFarRed extends LinearOpMode {
    //CHANGE SO THE LIST CHECKS ANY ACTION HAS BEEN CLICKED TWICE FOR EVERYTHING BESIDES SECRET TUNNEL AND HUMAN PLAYER
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
    public PathChain moveOffLine;
    public Boolean waiting = false;
    public ArrayList<Path> pathsToAddWait = new ArrayList<>();
    /* ---------------- OPMODE ---------------- */


    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(hardwareMap);

        /* -------- INIT BUTTON SELECTION -------- */

        while (opModeInInit()) {
            if(gamepad1.dpadUpWasReleased()) {
                if (!orderToPickup.isEmpty()) {
                    //wait logic
                    AutoAction lastAction = orderToPickup.get(orderToPickup.size() - 1); // get the last element of the order to pickup
                    if(lastAction == AutoAction.GATE_NO_INTAKE){
                        Log.i("ADDED WAITS: ", "GATE NO INTAKE");
                        gateNoIntake = waitHelper(moveToGateNoIntake, moveToShootGateNoIntake, Math.toRadians(0), gateNoIntake);
                    }
                    if (lastAction == AutoAction.FIRST_SPIKE) {
                        Log.i("ADDED WAITS: ", "FIRST SPIKE");
                        pickUpSpike1 = waitHelper(moveToSpike1Pickup, moveToShootSpike1, Math.toRadians(0), pickUpSpike1);
                    }
                    if (lastAction == AutoAction.SECOND_SPIKE) {
                        Log.i("ADDED WAITS: ", "SECOND SPIKE");
                        pickUpSpike2 = waitHelper(moveToSpike2Pickup, moveToShootSpike2, Math.toRadians(0), pickUpSpike2);
                    }
                    if (lastAction == AutoAction.GATE_PICKUP) {
                        Log.i("ADDED WAITS: ", "GATE PICKUP");
                        gatePickup = waitHelper(moveToGate, moveToShootGate, Math.toRadians(0), gatePickup);
                    }
                    if(lastAction == AutoAction.MOVE_OFF_LINE){
                        Log.i("ADDED WAITS: ", "MOVING OFF LINE");
                        moveOffLine = waitHelper(moveOffLaunchLine, Math.toRadians(0), moveOffLine);
                    }
                    if(lastAction == AutoAction.SECRET_TUNNEL && isSecretTunnelAfterGate()){
                        Log.i("ADDED WAITS: ", "SECRET TUNNEL AFTER GATE");
                        secretTunnelAfterGate = waitHelper(preSecretTunnelAfterGate, postSecretTunnel, Math.toRadians(0), secretTunnelAfterGate);
                    }
                    if(lastAction == AutoAction.SECRET_TUNNEL && !isSecretTunnelAfterGate()){
                        Log.i("ADDED WAITS: ", "SECRET TUNNEL AFTER SHOOTING");
                        secretTunnelAfterShot = waitHelper(moveToSecretTunnelAfterShooting, moveToShootAfterSecretTunnelShooting,SHOOTING_POSE.getHeading(), SECRET_TUNNEL_AFTER_SHOOTING.getHeading(), secretTunnelAfterShot);
                    }
                    if (lastAction == AutoAction.HUMAN_PLAYER) {
                        Log.i("WAITING: ", "HUMAN PLAYER");
                        pathsToAddWait.add(preHumanPlayer);
                        pathsToAddWait.add(moveToHumanPlayer);
                        pathsToAddWait.add(moveToShootHumanPlayer);
                        humanPlayerPickup = buildPathChainWithWait(pathsToAddWait, Math.toRadians(270), 0.9);
                        pathsToAddWait.clear();
                    }

                }
            }
            if(gamepad1.leftBumperWasReleased()) {
                Log.i("WAITING AT: ", "START OF PATH");
                if (!orderToPickup.isEmpty()) {
                    //wait logic
                    AutoAction lastAction = orderToPickup.get(orderToPickup.size() - 1); // get the last element of the order to pickup
                    if(lastAction == AutoAction.GATE_NO_INTAKE){
                        Log.i("ADDED WAITS TO START: ", "GATE NO INTAKE");
                        gateNoIntake = waitHelperStartWait(moveToGateNoIntake, moveToShootGateNoIntake, Math.toRadians(0), gateNoIntake);
                    }
                    if (lastAction == AutoAction.FIRST_SPIKE) {
                        Log.i("ADDED WAITS: ", "FIRST SPIKE");
                        pickUpSpike1 = waitHelperStartWait(moveToSpike1Pickup, moveToShootSpike1, Math.toRadians(0), pickUpSpike1);
                    }
                    if (lastAction == AutoAction.SECOND_SPIKE) {
                        Log.i("ADDED WAITS: ", "SECOND SPIKE");
                        pickUpSpike2 = waitHelperStartWait(moveToSpike2Pickup, moveToShootSpike2, Math.toRadians(0), pickUpSpike2);
                    }
                    if (lastAction == AutoAction.GATE_PICKUP) {
                        Log.i("ADDED WAITS: ", "GATE PICKUP");
                        gatePickup = waitHelperStartWait(moveToGate, moveToShootGate, Math.toRadians(0), gatePickup);
                    }
                    if(lastAction == AutoAction.SECRET_TUNNEL && isSecretTunnelAfterGate()){
                        Log.i("ADDED WAITS: ", "SECRET TUNNEL AFTER GATE");
                        secretTunnelAfterGate = waitHelperStartWait(preSecretTunnelAfterGate, postSecretTunnel, Math.toRadians(0), secretTunnelAfterGate);
                    }
                    if(lastAction == AutoAction.SECRET_TUNNEL && !isSecretTunnelAfterGate()){
                        Log.i("ADDED WAITS: ", "SECRET TUNNEL AFTER SHOOTING");
                        secretTunnelAfterShot = waitHelperStartWait(moveToSecretTunnelAfterShooting, moveToShootAfterSecretTunnelShooting,SHOOTING_POSE.getHeading(), SECRET_TUNNEL_AFTER_SHOOTING.getHeading(), secretTunnelAfterShot);
                    }
                    if (lastAction == AutoAction.HUMAN_PLAYER) {
                        Log.i("WAITING: ", "HUMAN PLAYER");
                        pathsToAddWait.add(preHumanPlayer);
                        pathsToAddWait.add(moveToHumanPlayer);
//                        pathsToAddWait.add(moveToShootHumanPlayer);
                        humanPlayerPickup = buildPathChainWithWaitStart(pathsToAddWait, Math.toRadians(270), 0.9, moveToShootHumanPlayer);
                        pathsToAddWait.clear();
                    }

                }
            }
            if (gamepad1.aWasReleased()) {
                orderToPickup.add(AutoAction.FIRST_SPIKE);
            }

            if (gamepad1.bWasReleased()) {
                addIfNotPresent(orderToPickup, AutoAction.SECOND_SPIKE);
            }

            if (gamepad1.yWasReleased()) {
                addIfNotPresent(orderToPickup, AutoAction.GATE_PICKUP);
            }

            if (gamepad1.xWasReleased()) {
                orderToPickup.add(AutoAction.HUMAN_PLAYER);
            }
            if(gamepad1.dpadDownWasReleased()){
                addIfNotPresent(orderToPickup, AutoAction.GATE_NO_INTAKE);
            }
            if(gamepad1.dpadLeftWasReleased()){
                addIfNotPresent(orderToPickup, AutoAction.MOVE_OFF_LINE);
            }
            if(gamepad1.dpadRightWasReleased()){
                orderToPickup.add(AutoAction.SECRET_TUNNEL);
            }

            if (orderToPickup.contains(AutoAction.FIRST_SPIKE) && orderToPickup.contains(AutoAction.HUMAN_PLAYER)) {
                if (orderToPickup.indexOf(AutoAction.FIRST_SPIKE) > orderToPickup.indexOf(AutoAction.HUMAN_PLAYER)) {
                    telemetry.addData("WARNING: ", "INTAKING HUMAN PLAYER BEFORE FIRST SPIKE");
                }
            }
            if (orderToPickup.contains(AutoAction.SECOND_SPIKE) && orderToPickup.contains(AutoAction.GATE_NO_INTAKE)) {
                if (orderToPickup.indexOf(AutoAction.SECOND_SPIKE) > orderToPickup.indexOf(AutoAction.GATE_NO_INTAKE)) {
                    telemetry.addData("WARNING: ", "OPENING GATE BEFORE SECOND SPIKE");
                }
            }
            if (orderToPickup.contains(AutoAction.SECOND_SPIKE) && orderToPickup.contains(AutoAction.GATE_PICKUP)) {
                if (orderToPickup.indexOf(AutoAction.SECOND_SPIKE) > orderToPickup.indexOf(AutoAction.GATE_PICKUP)) {
                    telemetry.addData("WARNING: ", "OPENING GATE BEFORE SECOND SPIKE");
                }
            }
            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderToPickup.toString());
            telemetry.update();
        }

        waitForStart();
        follower.setStartingPose(INIT_POSE);

        buildPaths();
        /* -------- AUTON LOOP -------- */

        while (opModeIsActive()) {
//            Log.i("WAITING METHOD CURRENT POSE: ", String.valueOf(follower.getPose()));
            if(!isMovedOffLineAtEnd){
                moveMoveOffLineToEnd();
                isMovedOffLineAtEnd = true;
            }
            if (waiting && waitTimer.seconds() >= waitSeconds) {
                Log.i("WAITING: ", "UNPAUSED");
                follower.resumePathFollowing();
                waiting = false;
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
        moveOffLine = follower.pathBuilder().addPath(moveOffLaunchLine).setLinearHeadingInterpolation(SHOOTING_POSE.getHeading(), MOVE_OFF_LINE.getHeading()).build();
        pickUpSpike1 = buildAPath(moveToSpike1Pickup, moveToShootSpike1, Math.toRadians(0));

        pickUpSpike2 = buildAPath(moveToSpike2Pickup, moveToShootSpike2, Math.toRadians(0));

        gatePickup = buildAPath(moveToGate, moveToShootGate, GATE_PICKUP.getHeading(), SHOOTING_POSE.getHeading());

        humanPlayerPickup = follower.pathBuilder()
                .addPath(preHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToShootHumanPlayer)
                .setLinearHeadingInterpolation(HUMAN_PLAYER_PICKUP.getHeading(), SHOOTING_POSE.getHeading())
                .build();

        gateNoIntake = buildAPath(moveToGate, moveToShootGateNoIntake, Math.toRadians(0));

        secretTunnelAfterGate = buildAPath(preSecretTunnelAfterGate, postSecretTunnel, Math.toRadians(0));
        gateNoShoot = follower.pathBuilder()
                .addPath(moveToGate)
                .setConstantHeadingInterpolation(Math.toRadians(0))
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
            Log.i("WAITING POSE: ", "STILL MOVING");
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
                Log.i("WAITING METHOD, ", "MOVING TO FIRST SPIKE");
                Log.i("WAITING METHOD, CURRENT POSE: ", String.valueOf(follower.getPose()));
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
                    Log.i("WAITING METHOD, ", "MOVING TO GATE");
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

    public PathChain buildPathChainWithWait(
            ArrayList<Path> paths,
            double startRad,
            double endRad,
            double whenToPause
    ) {
        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path list is null or empty");
        }

        PathBuilder builder = follower.pathBuilder();

        for (Path path : paths) {
            builder.addPath(path)
                    .setLinearHeadingInterpolation(startRad, endRad);
        }

        builder.addParametricCallback(
                whenToPause,
                this::pauseAndWait
        );

        return builder.build();
    }

    public PathChain buildPathChainWithWaitStart(
            ArrayList<Path> paths,
            double headingRad,
            double whenToPause,
            Path moveBackToShooting
    ) {
        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path list is null or empty");
        }

        PathBuilder builder = follower.pathBuilder();

        // Move TO the point
        for (Path path : paths) {
            builder.addPath(path)
                    .setConstantHeadingInterpolation(headingRad);
        }

        // Pause & wait at the destination
        builder.addParametricCallback(
                whenToPause,
                this::pauseAndWait
        );

        // AFTER wait → move back
        builder.addPath(moveBackToShooting)
                .setConstantHeadingInterpolation(headingRad);

        return builder.build();
    }

    public PathChain buildPathChainWithWaitStart(
            ArrayList<Path> paths,
            double startHeading,
            double endHeading,
            double whenToPause,
            Path moveBackToShooting
    ) {
        if (paths == null || paths.isEmpty()) {
            throw new IllegalArgumentException("Path list is null or empty");
        }

        PathBuilder builder = follower.pathBuilder();

        // Move TO the point
        for (Path path : paths) {
            builder.addPath(path)
                    .setLinearHeadingInterpolation(startHeading, endHeading);
        }

        // Pause & wait at the destination
        builder.addParametricCallback(
                whenToPause,
                this::pauseAndWait
        );

        // AFTER wait → move back
        builder.addPath(moveBackToShooting)
                .setLinearHeadingInterpolation(endHeading, startHeading);

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
    private void addIfNotPresent(
            ArrayList<AutoFarRed.AutoAction> list,
            AutoFarRed.AutoAction action
    ) {
        if (!list.contains(action)) {
            list.add(action);
        }
    }
    private PathChain waitHelper(Path moveToPath, Path shootSpike, double angRad, PathChain pathToWait){
        Log.i("WAITING: ", "HELPER");
        pathsToAddWait.add(moveToPath);
        pathsToAddWait.add(shootSpike);
        pathToWait = buildPathChainWithWait(pathsToAddWait, angRad, 0.9);
        pathsToAddWait.clear();
        return pathToWait;
    }
    private PathChain waitHelper(Path moveToPath, double angRad, PathChain pathToWait){
        Log.i("WAITING: ", "HELPER");
        pathsToAddWait.add(moveToPath);
        pathToWait = buildPathChainWithWait(pathsToAddWait, angRad, 0.9);
        pathsToAddWait.clear();
        return pathToWait;
    }
    private PathChain waitHelper(Path moveToPath, Path shootSpike, double startRad, double endRad, PathChain pathToWait){
        Log.i("WAITING: ", "HELPER");
        pathsToAddWait.add(moveToPath);
        pathsToAddWait.add(shootSpike);
        pathToWait = buildPathChainWithWait(pathsToAddWait, startRad, endRad, 0.9);
        pathsToAddWait.clear();
        return pathToWait;
    }
    private PathChain waitHelperStartWait(Path moveToPath, Path shootPose, double headingRad, PathChain pathToWait){
        Log.i("WAITING: ", "START HELPER");
        pathsToAddWait.add(moveToPath);
        pathToWait = buildPathChainWithWaitStart(pathsToAddWait, headingRad, 0.9, shootPose);
        pathsToAddWait.clear();
        return pathToWait;
    }
    private PathChain waitHelperStartWait(Path moveToPath, Path shootPose, double startHeading, double endHeading, PathChain pathToWait){
        Log.i("WAITING: ", "START HELPER");
        pathsToAddWait.add(moveToPath);
        pathToWait = buildPathChainWithWaitStart(pathsToAddWait, startHeading, endHeading, 0.9, shootPose);
        pathsToAddWait.clear();
        return pathToWait;
    }
}
