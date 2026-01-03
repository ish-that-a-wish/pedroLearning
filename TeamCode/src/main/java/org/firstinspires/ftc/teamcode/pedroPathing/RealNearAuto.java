package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.*;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.CallBacks.WaitCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

import java.util.ArrayList;

/**
 * RealNearAuto
 *
 * Main autonomous OpMode using PedroPathing.
 * Allows the driver to queue actions during INIT,
 * then executes them in order during AUTON.
 */
@Autonomous
public class RealNearAuto extends LinearOpMode {

    /* ------------------------------------------------------------
     * INIT SELECTION STATE
     * ------------------------------------------------------------ */

    // Prevents double-adding actions during init
    public boolean isFirstSpikePressed = false;
    public boolean isSecondSpikePressed = false;
    public boolean isGatePressed = false;
    public boolean isMovedOffLineAtEnd = false;

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
    private ArrayList<AutoAction> orderToPickup = new ArrayList<>();

    // Index of the currently executing action
    private int actionIndex = 0;

    // PedroPathing follower
    private Follower follower;

    // Timer used for wait callbacks
    private final ElapsedTime waitTimer = new ElapsedTime();

    // Length of wait when paused
    private double waitSeconds = 5.0;

    // True when robot is paused mid-path
    public Boolean waiting = false;

    // Temporary storage for paths that need waits injected
    public ArrayList<Path> pathsToAddWait = new ArrayList<>();

    /* ------------------------------------------------------------
     * POSES
     * ------------------------------------------------------------ */

    public Pose INIT_POSE = new Pose(20, 125, Math.toRadians(144));

    public static Pose FIRST_SPIKE  = new Pose(27, 84, Math.toRadians(180));
    public static Pose SECOND_SPIKE = new Pose(27, 60, Math.toRadians(180));
    public static Pose SHOOTING_POSE = new Pose(50, 84, Math.toRadians(180));

    public static Pose GATE_PICKUP = new Pose(14, 63, Math.toRadians(45));
    public static Pose GATE_PICKUP_NO_INTAKE = new Pose(10, 63, Math.toRadians(270));

    public static Pose PRE_HUMAN_PLAYER = new Pose(15, 30, Math.toRadians(270));
    public static Pose HUMAN_PLAYER_PICKUP = new Pose(15, 20, Math.toRadians(270));

    public Pose PRE_SECRET_TUNNEL = new Pose(3, 60, Math.toRadians(270));
    public Pose SECRET_TUNNEL_INTAKE = new Pose(3, 32, Math.toRadians(270));

    public Pose MOVE_OFF_LINE = new Pose(55, 115);

    /* Control points for curves */
    private Pose CONTROL_POINT_FIRST_SPIKE = new Pose(47, 82);
    private Pose CONTROL_POINT_SECOND_SPIKE = new Pose(55, 55);
    public Pose SECRET_TUNNEL_CONTROL_AFTER_GATE = new Pose(11, 57);

    /* ------------------------------------------------------------
     * PATH OBJECTS
     * ------------------------------------------------------------ */

    // Individual reusable paths

    public Path shootPreload =
            new Path(new BezierLine(INIT_POSE, SHOOTING_POSE));

    public Path moveToSpike1Pickup =
            new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE));

    public Path moveToShootSpike1 =
            new Path(new BezierLine(FIRST_SPIKE, SHOOTING_POSE));

    public Path moveToSpike2Pickup =
            new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE));

    public Path moveToShootSpike2 =
            new Path(new BezierLine(SECOND_SPIKE, SHOOTING_POSE));

    public Path moveToGate =
            new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP));

    public Path moveToShootGate =
            new Path(new BezierLine(GATE_PICKUP, SHOOTING_POSE));

    public Path preHumanPlayer =
            new Path(new BezierLine(SHOOTING_POSE, PRE_HUMAN_PLAYER));

    public Path moveToHumanPlayer =
            new Path(new BezierLine(PRE_HUMAN_PLAYER, HUMAN_PLAYER_PICKUP));

    public Path moveToShootHumanPlayer =
            new Path(new BezierLine(HUMAN_PLAYER_PICKUP, SHOOTING_POSE));

    public Path moveToGateNoIntake =
            new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP_NO_INTAKE));

    public Path moveToShootGateNoIntake =
            new Path(new BezierLine(GATE_PICKUP_NO_INTAKE, SHOOTING_POSE));

    public Path moveOffLaunchLine =
            new Path(new BezierLine(SHOOTING_POSE, MOVE_OFF_LINE));

    public Path preSecretTunnelAfterGate =
            new Path(new BezierCurve(
                    GATE_PICKUP_NO_INTAKE,
                    SECRET_TUNNEL_CONTROL_AFTER_GATE,
                    PRE_SECRET_TUNNEL
            ));

    public Path secretTunnelIntake =
            new Path(new BezierLine(PRE_SECRET_TUNNEL, SECRET_TUNNEL_INTAKE));

    public Path postSecretTunnel =
            new Path(new BezierLine(SECRET_TUNNEL_INTAKE, SHOOTING_POSE));

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

    /* ------------------------------------------------------------
     * OPMODE
     * ------------------------------------------------------------ */

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize follower and starting pose
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        // Build default paths
        buildPaths();

        /* ---------------- INIT LOOP ---------------- */
        while (opModeInInit()) {

            /*
             * DPAD UP:
             * Adds a WAIT to the most recently added action
             */
            if (gamepad1.dpadUpWasReleased()) {
                if (!orderToPickup.isEmpty()) {

                    AutoAction lastAction =
                            orderToPickup.get(orderToPickup.size() - 1);

                    if (lastAction == AutoAction.GATE_NO_INTAKE) {
                        pathsToAddWait.add(moveToGateNoIntake);
                        pathsToAddWait.add(moveToShootGateNoIntake);
                        gateNoIntake = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(270),
                                0.9
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.FIRST_SPIKE) {
                        pathsToAddWait.add(moveToSpike1Pickup);
                        pathsToAddWait.add(moveToShootSpike1);
                        pickUpSpike1 = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                0.9
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.SECOND_SPIKE) {
                        pathsToAddWait.add(moveToSpike2Pickup);
                        pathsToAddWait.add(moveToShootSpike2);
                        pickUpSpike2 = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                0.9
                        );
                        pathsToAddWait.clear();
                    }

                    if (lastAction == AutoAction.GATE_PICKUP) {
                        pathsToAddWait.add(moveToGate);
                        pathsToAddWait.add(moveToShootGate);
                        gatePickup = buildPathChainWithWait(
                                pathsToAddWait,
                                Math.toRadians(180),
                                0.9
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
                                0.9
                        );
                        pathsToAddWait.clear();
                    }
                }
            }

            /* ---------------- BUTTON MAPPING ---------------- */

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

            if (gamepad1.dpadDownWasReleased()) {
                orderToPickup.add(AutoAction.GATE_NO_INTAKE);
            }

            if (gamepad1.dpadLeftWasReleased()) {
                orderToPickup.add(AutoAction.MOVE_OFF_LINE);
            }

            if (gamepad1.dpadRightWasReleased()) {
                orderToPickup.add(AutoAction.SECRET_TUNNEL);
            }

            /* ---------------- TELEMETRY ---------------- */

            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderToPickup.toString());
            telemetry.update();
        }

        if (!orderToPickup.contains(AutoAction.SHOOTING_PRELOADS)) {
            orderToPickup.add(0, AutoAction.SHOOTING_PRELOADS);
        }

        waitForStart();

        /* ---------------- AUTON LOOP ---------------- */
        while (opModeIsActive()) {

            enforcePreloadFirst();

            // Ensure MOVE_OFF_LINE runs last
            if (!isMovedOffLineAtEnd) {
                moveMoveOffLineToEnd();
                isMovedOffLineAtEnd = true;
            }

            // Resume after wait
            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
            }

            follower.update();
            runPaths();

            telemetry.addData("ACTION ORDER", orderToPickup);
            telemetry.addData("Current Action Index", actionIndex);
            telemetry.update();
        }
    }

    /* ------------------------------------------------------------
     * PATH BUILDING
     * ------------------------------------------------------------ */

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
                .addPath(moveToGate)
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

        secretTunnel = follower.pathBuilder()
                .addPath(preSecretTunnelAfterGate)
                .setTangentHeadingInterpolation()
                .addPath(secretTunnelIntake)
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(postSecretTunnel)
                .setLinearHeadingInterpolation(
                        SECRET_TUNNEL_INTAKE.getHeading(),
                        SHOOTING_POSE.getHeading()
                )
                .build();

        preloadShot = follower.pathBuilder()
                .addPath(shootPreload)
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

    }

    /* ------------------------------------------------------------
     * STATE MACHINE
     * ------------------------------------------------------------ */

    private void runPaths() {

        // No more actions
        if (actionIndex >= orderToPickup.size()) return;

        // Wait until follower is free
        if (follower.isBusy()) return;
        if (waiting) return;

        AutoAction currentAction = orderToPickup.get(actionIndex);

        switch (currentAction) {

            case SHOOTING_PRELOADS:
                follower.followPath(preloadShot);
                break;


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
                if (isSecretTunnelAfterGate()) {
                    follower.followPath(moveToGateNoIntake);
                } else {
                    follower.followPath(gateNoIntake);
                }
                break;

            case SECRET_TUNNEL:
                follower.followPath(secretTunnel);
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
    }

    /* ------------------------------------------------------------
     * QUEUE HELPERS
     * ------------------------------------------------------------ */

    public void moveMoveOffLineToEnd() {
        if (orderToPickup.contains(AutoAction.MOVE_OFF_LINE)) {
            orderToPickup.remove(AutoAction.MOVE_OFF_LINE);
            orderToPickup.add(AutoAction.MOVE_OFF_LINE);
        }
    }

    public boolean isSecretTunnelAfterGate() {
        int gateNoIntakeIndex =
                orderToPickup.indexOf(AutoAction.GATE_NO_INTAKE);
        int gateIntakeIndex =
                orderToPickup.indexOf(AutoAction.GATE_PICKUP);
        int secretTunnelIndex =
                orderToPickup.indexOf(AutoAction.SECRET_TUNNEL);

        return secretTunnelIndex - 1 == gateNoIntakeIndex
                || secretTunnelIndex - 1 == gateIntakeIndex;
    }

    private void enforcePreloadFirst() {
        orderToPickup.remove(AutoAction.SHOOTING_PRELOADS);
        orderToPickup.add(0, AutoAction.SHOOTING_PRELOADS);
    }

}
