package org.firstinspires.ftc.teamcode.pedroPathing.Far;

import static org.firstinspires.ftc.teamcode.pedroPathing.Far.BlueFarConstants.*;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

import java.util.ArrayList;
import java.util.HashMap;

@Autonomous
public class BlueFarAuto extends LinearOpMode {

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

    /* ---------------- STATE ---------------- */

    private final ArrayList<AutoAction> orderToPickup = new ArrayList<>();
    private int actionIndex = 0;

    private Follower follower;

    /* ---------------- WAIT SYSTEM ---------------- */

    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean waiting = false;
    private double waitSeconds = 0.0;

    private final HashMap<AutoAction, Double> actionWaitTimes = new HashMap<>();
    private final HashMap<AutoAction, Boolean> hasMidWait = new HashMap<>();

    /* ---------------- PATHS ---------------- */

    private PathChain pickUpSpike1;
    private PathChain pickUpSpike2;
    private PathChain gatePickup;
    private PathChain humanPlayerPickup;
    private PathChain gateNoIntake;
    private PathChain secretTunnelAfterGate;
    private PathChain secretTunnelAfterShot;
    private PathChain moveOffLine;

    private boolean isMovedOffLineAtEnd = false;

    /* ---------------- OPMODE ---------------- */

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        buildPaths();

        /* ================= INIT ================= */

        while (opModeInInit()) {

            /* ---- WAIT ADJUST ---- */
            if (!orderToPickup.isEmpty()) {
                AutoAction last = orderToPickup.get(orderToPickup.size() - 1);

                if (gamepad1.rightBumperWasReleased()) {
                    actionWaitTimes.put(
                            last,
                            actionWaitTimes.getOrDefault(last, 0.0) + 1.0
                    );
                }

                if (gamepad1.leftBumperWasReleased()) {
                    actionWaitTimes.put(
                            last,
                            Math.max(0.0,
                                    actionWaitTimes.getOrDefault(last, 0.0) - 1.0)
                    );
                }

                /* ---- MID WAIT ADD ---- */
//                if (gamepad1.leftStickButtonWasReleased()) {
//                    Log.i("Pressed button: ", "Left Stick");
//                    hasMidWait.put(last, true);
//                    rebuildPathWithMidWait(last);
//                }
            }

            /* ---- ACTION SELECTION ---- */
            if (gamepad1.aWasReleased()) addIfNotPresent(AutoAction.FIRST_SPIKE);
            if (gamepad1.bWasReleased()) addIfNotPresent(AutoAction.SECOND_SPIKE);
            if (gamepad1.yWasReleased()) addIfNotPresent(AutoAction.GATE_PICKUP);
            if (gamepad1.xWasReleased()) orderToPickup.add(AutoAction.HUMAN_PLAYER);
            if (gamepad1.dpadDownWasReleased()) addIfNotPresent(AutoAction.GATE_NO_INTAKE);
            if (gamepad1.dpadLeftWasReleased()) addIfNotPresent(AutoAction.MOVE_OFF_LINE);
            if (gamepad1.dpadRightWasReleased()) orderToPickup.add(AutoAction.SECRET_TUNNEL);

            /* ---- TELEMETRY ---- */
            telemetry.addLine("INIT CONFIG");
            telemetry.addData("Queue", orderToPickup);

            if (!orderToPickup.isEmpty()) {
                AutoAction last = orderToPickup.get(orderToPickup.size() - 1);
                telemetry.addData("Wait", actionWaitTimes.getOrDefault(last, 0.0));
                telemetry.addData("Has Mid Wait", hasMidWait.getOrDefault(last, false));
            }

            telemetry.update();
        }

        waitForStart();

        /* ================= AUTON ================= */

        while (opModeIsActive()) {

            if (!isMovedOffLineAtEnd) {
                moveMoveOffLineToEnd();
                isMovedOffLineAtEnd = true;
            }

            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
            }

            follower.update();
            runPaths();

            telemetry.addLine("AUTON");
            telemetry.addData("Action Index", actionIndex);
            telemetry.addData("Waiting", waiting);
            telemetry.addData("Wait Seconds", waitSeconds);
            telemetry.update();
        }
    }

    /* ---------------- PATH BUILDING ---------------- */

    private void buildPaths() {

        moveOffLine = follower.pathBuilder()
                .addPath(moveOffLaunchLine)
                .setLinearHeadingInterpolation(
                        SHOOTING_POSE.getHeading(),
                        MOVE_OFF_LINE.getHeading()
                )
                .build();

        pickUpSpike1 = buildStandardPath(
                AutoAction.FIRST_SPIKE,
                moveToSpike1Pickup,
                moveToShootSpike1,
                Math.toRadians(180)
        );

        pickUpSpike2 = buildStandardPath(
                AutoAction.SECOND_SPIKE,
                moveToSpike2Pickup,
                moveToShootSpike2,
                Math.toRadians(180)
        );

        gatePickup = buildStandardPath(
                AutoAction.GATE_PICKUP,
                moveToGate,
                moveToShootGate,
                GATE_PICKUP.getHeading(),
                SHOOTING_POSE.getHeading()
        );

        gateNoIntake = buildStandardPath(
                AutoAction.GATE_NO_INTAKE,
                moveToGate,
                moveToShootGateNoIntake,
                Math.toRadians(180)
        );

        secretTunnelAfterGate = buildStandardPath(
                AutoAction.SECRET_TUNNEL,
                preSecretTunnelAfterGate,
                postSecretTunnel,
                Math.toRadians(180)
        );

        secretTunnelAfterShot = buildStandardPath(
                AutoAction.SECRET_TUNNEL,
                moveToSecretTunnelAfterShooting,
                moveToShootAfterSecretTunnelShooting,
                SECRET_TUNNEL_AFTER_SHOOTING.getHeading(),
                SHOOTING_POSE.getHeading()
        );
    }

    /* ---------------- STATE MACHINE ---------------- */

    private void runPaths() {

        if (actionIndex >= orderToPickup.size()) return;
        if (follower.isBusy() || waiting) return;

        AutoAction action = orderToPickup.get(actionIndex);

        switch (action) {
            case MOVE_OFF_LINE:
                follower.followPath(moveOffLine);
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
            case GATE_NO_INTAKE:
                follower.followPath(gateNoIntake);
                break;
            case SECRET_TUNNEL:
                follower.followPath(
                        isSecretTunnelAfterGate()
                                ? secretTunnelAfterGate
                                : secretTunnelAfterShot
                );
                break;
        }

        actionIndex++;
    }

    /* ---------------- WAIT ---------------- */

    private void pauseAndWait(AutoAction action) {
        follower.pausePathFollowing();
        waitTimer.reset();
        waiting = true;
        waitSeconds = actionWaitTimes.getOrDefault(action, 0.0);
        Log.i("WAIT", action + " : " + waitSeconds);
    }

    /* ---------------- PATH HELPERS ---------------- */

    private PathChain buildStandardPath(
            AutoAction action,
            Path move,
            Path shoot,
            double heading
    ) {
        PathBuilder builder = follower.pathBuilder()
                .addPath(move)
                .setConstantHeadingInterpolation(heading);

        if (hasMidWait.getOrDefault(action, false)) {
            builder.addParametricCallback(
                    0.99,
                    () -> pauseAndWait(action)
            );
        }

        builder.addPath(shoot)
                .setConstantHeadingInterpolation(heading)
                .addParametricCallback(
                        0.9,
                        () -> pauseAndWait(action)
                );

        return builder.build();
    }

    private PathChain buildStandardPath(
            AutoAction action,
            Path move,
            Path shoot,
            double start,
            double end
    ) {
        PathBuilder builder = follower.pathBuilder()
                .addPath(move)
                .setLinearHeadingInterpolation(end, start);

        if (hasMidWait.getOrDefault(action, false)) {
            builder.addParametricCallback(
                    0.99,
                    () -> pauseAndWait(action)
            );
        }

        builder.addPath(shoot)
                .setLinearHeadingInterpolation(start, end)
                .addParametricCallback(
                        0.9,
                        () -> pauseAndWait(action)
                );

        return builder.build();
    }

    private void rebuildPathWithMidWait(AutoAction action) {
        buildPaths();
    }

    /* ---------------- HELPERS ---------------- */

    private boolean isSecretTunnelAfterGate() {
        int gate = orderToPickup.indexOf(AutoAction.GATE_NO_INTAKE);
        int tunnel = orderToPickup.indexOf(AutoAction.SECRET_TUNNEL);
        return gate != -1 && tunnel == gate + 1;
    }

    private void moveMoveOffLineToEnd() {
        if (orderToPickup.remove(AutoAction.MOVE_OFF_LINE)) {
            orderToPickup.add(AutoAction.MOVE_OFF_LINE);
        }
    }

    private void addIfNotPresent(AutoAction action) {
        if (!orderToPickup.contains(action)) {
            orderToPickup.add(action);
        }
    }
}

