package org.firstinspires.ftc.teamcode.AutoWithRR;

import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.GATE_PICKUP;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.GATE_PICKUP_NO_INTAKE;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.HUMAN_PLAYER_PICKUP;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.INIT_POSE;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.SHOOTING_POSE;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.SecretTunnel;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.SecretTunnelAfterGate;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveOffLaunchLine;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToGate;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToGateNoIntake;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToHumanPlayer;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootGate;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootGateNoIntake;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootHumanPlayer;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootSpike1;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootSpike2;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToSpike1Pickup;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToSpike2Pickup;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.postSecretTunnel;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.preHumanPlayer;
import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.shootPreload;

import android.util.Log;

import org.firstinspires.ftc.teamcode.common.SpindexAutoAdvanceCommand;
import org.firstinspires.ftc.teamcode.common.intakeCommand;

import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.spindexCommandReal;
import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;

import java.util.ArrayList;
import java.util.HashMap;

/**
 * RealNearAuto
 *
 * Main autonomous OpMode using PedroPathing.
 * Allows the driver to queue actions during INIT,
 * then executes them in order during AUTON.
 */
@Autonomous(name="BlueNearWithActions")
public class BlueNearAuto extends LinearOpMode {
    public boolean init = true;

    public RobotHardware robotHardware;
    /* ------------------------------------------------------------
     * INIT SELECTION STATE
     * ------------------------------------------------------------ */

    // Prevents double-adding actions during init
    public boolean isFirstSpikePressed = false;
    public boolean isSecondSpikePressed = false;
    public boolean isGatePressed = false;
    public boolean isMovedOffLineAtEnd = false;
    public boolean shotPreloads = false;

    /* ------------------------------------------------------------
     * ACTION ENUM
     * ------------------------------------------------------------ */

    public enum AutoAction {
        FIRST_SPIKE,
        SECOND_SPIKE,
        GATE_PICKUP,
        HUMAN_PLAYER,
        GATE_NO_INTAKE,
        MOVE_OFF_LINE,
        SECRET_TUNNEL,

        SHOOTING_PRELOADS
    }

    /* ------------------------------------------------------------
     * CORE FIELDS
     * ------------------------------------------------------------ */

    // Ordered list of actions chosen during init
    private ArrayList<AutoAction> orderedListOfPathsWithActions = new ArrayList<>();

    // Index of the currently executing action
    private int actionIndex = 0;

    // PedroPathing follower
    private Follower follower;

    // Timer used for wait callbacks
    private final ElapsedTime waitTimer = new ElapsedTime();

    // Length of wait when paused
    private double waitSeconds = 0.0; //5.0

    // True when robot is paused mid-path
    public Boolean waiting = false;

    // Temporary storage for paths that need waits injected
    public ArrayList<Path> pathsToAddWait = new ArrayList<>();

    /* ------------------------------------------------------------
     * POSES
     * ------------------------------------------------------------ */

    /* ------------------------------------------------------------
     * PATH CHAINS
     * ------------------------------------------------------------ */

    private PathChain pickUpSpike1;
    private PathChain pickUpSpike2;
    private PathChain gatePickup;
    private PathChain humanPlayerPickup;
    public PathChain gateNoIntake;
    public PathChain secretTunnel;
    public PathChain gateNoShoot;

    public PathChain preloadShot;

    public PathChain tunnelAfterGate;


    // Optional custom wait times for specific actions
    private final HashMap<AutoAction, Double> actionWaitTimes = new HashMap<>();

    private AutoAction currentActionForWait = null;

    private intakeCommand intakeStartCommand;
    private intakeCommand intakeStopCommand;

    private SpindexSubsystem spindexSubsystem;

    private intakeSubsystem intakeSubsystem;
    private spindexCommandReal spindexCommandReal;

    private Spindex rrSpindex;



    /* ------------------------------------------------------------
     * OPMODE
     * ------------------------------------------------------------ */

    @Override
    public void runOpMode() throws InterruptedException {


        robotHardware = new RobotHardware(this.hardwareMap);

        // Initialize follower and starting pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);
        intakeSubsystem = new intakeSubsystem(this.hardwareMap);

        intakeStartCommand = new intakeCommand(intakeSubsystem, true);
        intakeStopCommand = new intakeCommand(intakeSubsystem, false);

//        spindexSubsystem = new SpindexSubsystem(robotHardware, this.follower);
        spindexCommandReal = new spindexCommandReal(spindexSubsystem, robotHardware, follower);

        rrSpindex = new Spindex(robotHardware);

        // Build default paths
        buildPaths();

        CommandScheduler.getInstance().reset();

        /* ---------------- INIT LOOP ---------------- */
        while (opModeInInit()) {

//            spindexSubsystem.initializeWithEmpty(); // or whatever config
//            spindexSubsystem.setCurrentIndex(0);

            /*
             * DPAD UP:
             * Adds a WAIT to the most recently added action
             */

            if (gamepad1.dpadUpWasReleased()) {
                if (!orderedListOfPathsWithActions.isEmpty()) {

                    AutoAction lastAction =
                            orderedListOfPathsWithActions.get(orderedListOfPathsWithActions.size() - 1);

                    if (lastAction == AutoAction.GATE_NO_INTAKE) {
                        pathsToAddWait.add(moveToGateNoIntake);
                        pathsToAddWait.add(moveToShootGateNoIntake);
                        gateNoIntake = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(270),
                                1
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.FIRST_SPIKE) {
                        pathsToAddWait.add(moveToSpike1Pickup);
                        pathsToAddWait.add(moveToShootSpike1);
                        pickUpSpike1 = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                1
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.SECOND_SPIKE) {
                        pathsToAddWait.add(moveToSpike2Pickup);
                        pathsToAddWait.add(moveToShootSpike2);
                        pickUpSpike2 = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                1
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.GATE_PICKUP) {
                        pathsToAddWait.add(moveToGate);
                        pathsToAddWait.add(moveToShootGate);
                        gatePickup = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                1
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.HUMAN_PLAYER) {
                        pathsToAddWait.add(preHumanPlayer);
                        pathsToAddWait.add(moveToHumanPlayer);
                        pathsToAddWait.add(moveToShootHumanPlayer);
                        humanPlayerPickup = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(270),
                                1
                        );



                        pathsToAddWait.clear();
                    }
                }
            }

            /* ---------------- BUTTON MAPPING ---------------- */

            // Right trigger: increase wait
            if (gamepad1.rightBumperWasReleased()) {
                if (!orderedListOfPathsWithActions.isEmpty()) {
                    AutoAction lastAction = orderedListOfPathsWithActions.get(orderedListOfPathsWithActions.size() - 1);
                    double currentWait = actionWaitTimes.getOrDefault(lastAction, 0.0);
                    currentWait += 1.0; // increment by 1 second
                    actionWaitTimes.put(lastAction, currentWait);

                    rebuildPathChainForAction(lastAction);
                }
            }

// Left trigger: decrease wait
            if (gamepad1.leftBumperWasReleased()) {
                if (!orderedListOfPathsWithActions.isEmpty()) {
                    AutoAction lastAction = orderedListOfPathsWithActions.get(orderedListOfPathsWithActions.size() - 1);
                    double currentWait = actionWaitTimes.getOrDefault(lastAction, 0.0);
                    currentWait = Math.max(0.0, currentWait - 1.0);
                    actionWaitTimes.put(lastAction, currentWait);

                    rebuildPathChainForAction(lastAction);
                }
            }


            if (gamepad1.aWasReleased() && !isFirstSpikePressed) {
                orderedListOfPathsWithActions.add(AutoAction.FIRST_SPIKE);
                isFirstSpikePressed = true;
            }

            if (gamepad1.bWasReleased() && !isSecondSpikePressed) {
                orderedListOfPathsWithActions.add(AutoAction.SECOND_SPIKE);
                isSecondSpikePressed = true;
            }

            if (gamepad1.yWasReleased() && !isGatePressed) {
                orderedListOfPathsWithActions.add(AutoAction.GATE_PICKUP);
                isGatePressed = true;
            }

            if (gamepad1.xWasReleased()) {
                orderedListOfPathsWithActions.add(AutoAction.HUMAN_PLAYER);
            }

            if (gamepad1.dpadDownWasReleased()) {
                orderedListOfPathsWithActions.add(AutoAction.GATE_NO_INTAKE);
            }

            if (gamepad1.dpadLeftWasReleased()) {
                orderedListOfPathsWithActions.add(AutoAction.MOVE_OFF_LINE);
            }

            if (gamepad1.dpadRightWasReleased()) {
                orderedListOfPathsWithActions.add(AutoAction.SECRET_TUNNEL);
            }

            /* ---------------- TELEMETRY ---------------- */

            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderedListOfPathsWithActions.toString());

            if (!orderedListOfPathsWithActions.isEmpty()) {
                AutoAction lastAction = orderedListOfPathsWithActions.get(orderedListOfPathsWithActions.size() - 1);
                double waitForLast = actionWaitTimes.getOrDefault(lastAction, 0.0);
                telemetry.addData("Wait for last action", waitForLast + "s");
            }

            telemetry.update();
            telemetry.update();
        }

        if (!orderedListOfPathsWithActions.contains(AutoAction.SHOOTING_PRELOADS)) {
            orderedListOfPathsWithActions.add(0, AutoAction.SHOOTING_PRELOADS);
        }

        waitForStart();

        /* ---------------- AUTON LOOP ---------------- */
        while (opModeIsActive()) {
            CommandScheduler.getInstance().run();

            enforcePreloadFirst();


//            Actions.runBlocking(intakeSystem.checkForBallIntakeAndGetAction());
            // Ensure MOVE_OFF_LINE runs last
            if (!isMovedOffLineAtEnd) {
                moveOffLineAtEnd();
                isMovedOffLineAtEnd = true;
            }

            // Resume after wait
            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
            }
//            if(startIntake){
//                intakeSystem.checkForBallIntakeAndGetAction();
//            }
            follower.update();
            runPaths();

            telemetry.addData("ACTION ORDER", orderedListOfPathsWithActions);
            telemetry.addData("Current Action Index", actionIndex);
            telemetry.update();
        }
    }

    /* ------------------------------------------------------------
     * PATH BUILDING
     * ------------------------------------------------------------ */

    private void spindexMotion(){
        rrSpindex.initializeWithUnknowns();
        while(!rrSpindex.isFull()){
            while (!robotHardware.isSpindexBusy()){
                if(robotHardware.didBallDetectionBeamBreak()){
                    Log.i("RRSPINDEX", "RUNNING");
                    Actions.runBlocking(
                            rrSpindex.moveToNextEmptySlotAction()
                    );
                }
            }
        }

    }

    private void buildPaths() {

        pickUpSpike1 = follower.pathBuilder()
                .addPath(moveToSpike1Pickup)
                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .addParametricCallback(0.1, this::startIntake)
                //.addParametricCallback(0.1, ()-> CommandScheduler.getInstance().schedule(intakeStartCommand))
//                .addParametricCallback(0.15, () ->
//                        CommandScheduler.getInstance().schedule(
//                                new SpindexAutoAdvanceCommand(spindexSubsystem, 10, robotHardware, rrSpindex)
//                        )
//                )

                .addParametricCallback(0.15, this::spindexMotion)

                //.addParametricCallback(0.15, ()-> CommandScheduler.getInstance().schedule(spindexCommandReal))
                //.addParametricCallback(0.1, ()-> CommandScheduler.getInstance().schedule(intakeCommand1))
                .addPath(moveToShootSpike1)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickUpSpike2 = follower.pathBuilder()
                .addPath(moveToSpike2Pickup)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.1, ()-> CommandScheduler.getInstance().schedule(intakeStartCommand))
                .build();
//                .addParameck(0.15, ()-> CommandScheduler.getInstance().schedule(spindexCommandReal))
//                .addPath(moveToShootSpike2)
//                .setConstantHeadingInterpolation(Math.toRadians(180))
//                .build();tricCallback(0.15, ()-> CommandScheduler.getInstance().schedule(spindexSubsystem.moveToNextEmptySlotCommand()))


        gatePickup = follower.pathBuilder()
                .addPath(moveToGate)
                .setLinearHeadingInterpolation(
                        SHOOTING_POSE.getHeading(),
                        GATE_PICKUP.getHeading()
                )
                .addPath(moveToShootGate)
                .setLinearHeadingInterpolation(
                        GATE_PICKUP.getHeading(),
                        SHOOTING_POSE.getHeading()
                )
                .build();

        humanPlayerPickup = follower.pathBuilder()
                .addPath(preHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToHumanPlayer)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(moveToShootHumanPlayer)
                .setLinearHeadingInterpolation(
                        HUMAN_PLAYER_PICKUP.getHeading(),
                        SHOOTING_POSE.getHeading()
                )
                .build();

        gateNoIntake = follower.pathBuilder()
                .addPath(moveToGateNoIntake)
                .setLinearHeadingInterpolation(
                        SHOOTING_POSE.getHeading(),
                        GATE_PICKUP_NO_INTAKE.getHeading()
                )
                .addPath(moveToShootGateNoIntake)
                .setLinearHeadingInterpolation(
                        GATE_PICKUP_NO_INTAKE.getHeading(),
                        SHOOTING_POSE.getHeading()
                )
                .build();

        secretTunnel =
                follower.pathBuilder()
                        .addPath(SecretTunnel)
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .addPath(postSecretTunnel)
                        .setConstantHeadingInterpolation(Math.toRadians(180))
                        .build();

        preloadShot = follower.pathBuilder()
                .addPath(shootPreload)
//                .addParametricCallback(0.01, () ->
//                        CommandScheduler.getInstance().schedule(
//                                new SpindexIndexOnceCommand(spindexSubsystem)
//                        )
//                )
//                .addParametricCallback(0.1, ()->Actions.runBlocking(intakeSystem.getTurnOnAction()))
//                .addParametricCallback(0.1, ()->spindex.initializeWithUnknowns())
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        tunnelAfterGate = follower.pathBuilder()
                .addPath(SecretTunnelAfterGate)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(postSecretTunnel)
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();


    }

    /* ------------------------------------------------------------
     * STATE MACHINE
     * ------------------------------------------------------------ */

    private void runPaths() {

        // No more actions
        if (actionIndex >= orderedListOfPathsWithActions.size()) return;

        // Wait until follower is free
        if (follower.isBusy()) return;
        if (waiting) return;

        // --- Track the action for wait callbacks ---
        AutoAction currentAction = orderedListOfPathsWithActions.get(actionIndex);
        currentActionForWait = currentAction;

        switch (currentAction) {

            case SHOOTING_PRELOADS:
                shotPreloads = true;
                follower.followPath(preloadShot);
                break;


            case MOVE_OFF_LINE:
                follower.followPath(moveOffLaunchLine);
                break;

            case FIRST_SPIKE:
                if(shotPreloads) {
                    follower.followPath(pickUpSpike1);
//                    Actions.runBlocking(intakeSystem.checkForBallIntakeAndGetAction());
                }
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
                if (isSecretTunnelAfterGate()) {
                    follower.followPath(moveToGateNoIntake);
                } else {
                    follower.followPath(gateNoIntake);
                }
                break;

            case SECRET_TUNNEL:
                if(!isSecretTunnelAfterGate()) {
                    follower.followPath(secretTunnel);
                }
                if(isSecretTunnelAfterGate()){
                    follower.followPath(tunnelAfterGate);
                }
                break;

        }

        // Advance to next action
        actionIndex++;
    }

    /* ------------------------------------------------------------
     * WAIT CALLBACK UTILITIES
     * ------------------------------------------------------------ */

    public PathChain buildPathChainWithWait(
            ArrayList<Path> paths,
            double headingRadians,
            double whenToPause
    ) {

        PathBuilder builder = follower.pathBuilder();

        for (Path path : paths) {
            builder.addPath(path)
                    .setConstantHeadingInterpolation(headingRadians);
        }

        // Pause once at the given parametric position
        builder.addParametricCallback(whenToPause, this::pauseAndWait);

        return builder.build();
    }

    public void pauseAndWait() {
        follower.pausePathFollowing();
        waitTimer.reset();
        waiting = true;

        // Only use the action that triggered this path
        if (currentActionForWait != null && actionWaitTimes.containsKey(currentActionForWait)) {
            waitSeconds = actionWaitTimes.get(currentActionForWait);
        } else {
            waitSeconds = 0.0;
        }
    }

//    public void pauseAndWait() {
//        follower.pausePathFollowing();
//        waitTimer.reset();
//        waiting = true;
//
//        // Use custom wait for the action that triggered this path
//        if (currentActionForWait != null && actionWaitTimes.containsKey(currentActionForWait)) {
//            waitSeconds = actionWaitTimes.get(currentActionForWait);
//        } else {
//            waitSeconds = 0.0;
//        }
//
//        // Use custom wait if defined for current action, otherwise 0
//        AutoAction currentAction = actionIndex < orderToPickup.size()
//                ? orderToPickup.get(actionIndex)
//                : null;
//
//        if (currentAction != null && actionWaitTimes.containsKey(currentAction)) {
//            waitSeconds = actionWaitTimes.get(currentAction);
//        } else {
//            waitSeconds = 0.0;  // default
//        }
//
//    }

    /* ------------------------------------------------------------
     * QUEUE HELPERS
     * ------------------------------------------------------------ */

    public void moveOffLineAtEnd() {
        if (orderedListOfPathsWithActions.contains(AutoAction.MOVE_OFF_LINE)) {
            orderedListOfPathsWithActions.remove(AutoAction.MOVE_OFF_LINE);
            orderedListOfPathsWithActions.add(AutoAction.MOVE_OFF_LINE);
        }
    }

    public boolean isSecretTunnelAfterGate() {
        int gateNoIntakeIndex =
                orderedListOfPathsWithActions.indexOf(AutoAction.GATE_NO_INTAKE);
        int gateIntakeIndex =
                orderedListOfPathsWithActions.indexOf(AutoAction.GATE_PICKUP);
        int secretTunnelIndex =
                orderedListOfPathsWithActions.indexOf(AutoAction.SECRET_TUNNEL);

        return secretTunnelIndex - 1 == gateNoIntakeIndex
                || secretTunnelIndex - 1 == gateIntakeIndex;
    }

    private void enforcePreloadFirst() {
        orderedListOfPathsWithActions.remove(AutoAction.SHOOTING_PRELOADS);
        orderedListOfPathsWithActions.add(0, AutoAction.SHOOTING_PRELOADS);
    }

    private void rebuildPathChainForAction(AutoAction action) {
        switch(action) {
            case FIRST_SPIKE:
                pathsToAddWait.add(moveToSpike1Pickup);
                pathsToAddWait.add(moveToShootSpike1);
                pickUpSpike1 = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 1);
                pathsToAddWait.clear();
                break;

            case SECOND_SPIKE:
                pathsToAddWait.add(moveToSpike2Pickup);
                pathsToAddWait.add(moveToShootSpike2);
                pickUpSpike2 = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 1);
                pathsToAddWait.clear();
                break;

            case GATE_PICKUP:
                pathsToAddWait.add(moveToGate);
                pathsToAddWait.add(moveToShootGate);
                gatePickup = buildPathChainWithWait(pathsToAddWait, Math.toRadians(180), 1);
                pathsToAddWait.clear();
                break;

            case HUMAN_PLAYER:
                pathsToAddWait.add(preHumanPlayer);
                pathsToAddWait.add(moveToHumanPlayer);
                pathsToAddWait.add(moveToShootHumanPlayer);
                humanPlayerPickup = buildPathChainWithWait(pathsToAddWait, Math.toRadians(270), 1);
                pathsToAddWait.clear();
                break;

            case GATE_NO_INTAKE:
                pathsToAddWait.add(moveToGateNoIntake);
                pathsToAddWait.add(moveToShootGateNoIntake);
                gateNoIntake = buildPathChainWithWait(pathsToAddWait, Math.toRadians(270), 1);
                pathsToAddWait.clear();
                break;
        }
    }
//    public void startIntake(){
//        startIntake = true;
//    }

}