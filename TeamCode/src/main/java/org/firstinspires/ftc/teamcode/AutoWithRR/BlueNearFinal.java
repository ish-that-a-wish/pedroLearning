package org.firstinspires.ftc.teamcode.AutoWithRR;

import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.*;

import com.pedropathing.follower.Follower;
import com.pedropathing.paths.*;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSystem;
import org.firstinspires.ftc.teamcode.subsystems.LaunchSystem;

import java.util.*;

@Autonomous(name = "Blue Near Final - Fixed", group = "Autonomous")
public class BlueNearFinal extends LinearOpMode {
    public static double SHOOT_HOLD_SECONDS = 3.0;

    /* ===================== SUBSYSTEMS ===================== */
    public IntakeSystem intakeSystem;
    public LaunchSystem launchSystem;

    /* ===================== ENUM ===================== */
    public enum AutoAction {
        SHOOTING_PRELOADS,
        FIRST_SPIKE,
        SECOND_SPIKE,
        GATE_PICKUP,
        HUMAN_PLAYER,
        GATE_NO_INTAKE,
        SECRET_TUNNEL,
        MOVE_OFF_LINE
    }

    /* ===================== CORE STATE ===================== */
    private final List<AutoAction> actionQueue = new ArrayList<>();
    private final Map<AutoAction, Double> actionWaitTimes = new HashMap<>();

    private final ElapsedTime waitTimer = new ElapsedTime();
    private boolean waiting = false;
    private double waitSeconds = 0;

    private int actionIndex = 0;
    private AutoAction currentActionForWait = null;

    private Follower follower;

    // Rising Edge Detection States
    private boolean lastA, lastB, lastX, lastY, lastUp, lastDown, lastLeft, lastRight, lastLB, lastRB;

    /* ===================== PATH CHAINS ===================== */
    private PathChain spike1, spike2, gatePickup, gateNoIntake;
    private PathChain humanPlayer, secretTunnel, tunnelAfterGate, preloadShot;

    @Override
    public void runOpMode() {
        // Fix 5: Ensure these are initialized before use in buildPaths()
        // (Leaving your existing initialization style as requested)
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        buildPaths();

        /* ---------- INIT LOOP ---------- */
        while (opModeInInit()) {
            handleInitControls();
            sendTelemetry();
        }

        // Fix 3: Organize queue once before start to prevent infinite loops
        enforcePreloadFirst();
        moveMoveOffLineToEnd();

        waitForStart();

        /* ---------- AUTON LOOP ---------- */
        while (opModeIsActive()) {
            // Wait timer handling
            if (waiting && waitTimer.seconds() >= waitSeconds) {
                follower.resumePathFollowing();
                waiting = false;
            }

            follower.update();
            runPaths();

            telemetry.addData("Status", waiting ? "Waiting: " + (waitSeconds - waitTimer.seconds()) : "Following Path");
            telemetry.addData("Queue", actionQueue);
            telemetry.addData("Action Index", actionIndex);
            telemetry.update();
        }
    }

    /* ===================== INIT CONTROLS ===================== */
    private void handleInitControls() {
        // Fix: Standard Gamepad boolean fields with Rising Edge Detection
        if (gamepad1.a && !lastA) addOnce(AutoAction.FIRST_SPIKE);
        if (gamepad1.b && !lastB) addOnce(AutoAction.SECOND_SPIKE);
        if (gamepad1.y && !lastY) addOnce(AutoAction.GATE_PICKUP);
        if (gamepad1.x && !lastX) actionQueue.add(AutoAction.HUMAN_PLAYER);

        if (gamepad1.dpad_down && !lastDown) actionQueue.add(AutoAction.GATE_NO_INTAKE);
        if (gamepad1.dpad_right && !lastRight) actionQueue.add(AutoAction.SECRET_TUNNEL);
        if (gamepad1.dpad_left && !lastLeft) actionQueue.add(AutoAction.MOVE_OFF_LINE);

        if (gamepad1.right_bumper && !lastRB) adjustWait(+1);
        if (gamepad1.left_bumper && !lastLB) adjustWait(-1);

        // Update states
        lastA = gamepad1.a; lastB = gamepad1.b; lastX = gamepad1.x; lastY = gamepad1.y;
        lastDown = gamepad1.dpad_down; lastRight = gamepad1.dpad_right; lastLeft = gamepad1.dpad_left;
        lastLB = gamepad1.left_bumper; lastRB = gamepad1.right_bumper;
    }

    private void addOnce(AutoAction action) {
        if (!actionQueue.contains(action)) actionQueue.add(action);
    }

    private void adjustWait(int delta) {
        if (actionQueue.isEmpty()) return;
        AutoAction last = actionQueue.get(actionQueue.size() - 1);
        double newWait = Math.max(0, actionWaitTimes.getOrDefault(last, 0.0) + delta);
        actionWaitTimes.put(last, newWait);
    }

    /* ===================== PATH EXECUTION ===================== */
    private void runPaths() {
        if (actionIndex >= actionQueue.size()) return;
        if (follower.isBusy() || waiting) return;

        AutoAction action = actionQueue.get(actionIndex);
        currentActionForWait = action;

        // Fix 1: Added breaks to prevent fallthrough
        switch (action) {
            case SHOOTING_PRELOADS: follower.followPath(preloadShot); break;
            case FIRST_SPIKE: follower.followPath(spike1); break;
            case SECOND_SPIKE: follower.followPath(spike2); break;
            case GATE_PICKUP: follower.followPath(gatePickup); break;
            case HUMAN_PLAYER: follower.followPath(humanPlayer); break;
            case MOVE_OFF_LINE: follower.followPath(moveOffLaunchLine); break;
            case GATE_NO_INTAKE:
                if(isSecretTunnelAfterGate()){ follower.followPath(moveToGateNoIntake); }
                else { follower.followPath(gateNoIntake); }
                break;
            case SECRET_TUNNEL:
                follower.followPath(isSecretTunnelAfterGate() ? tunnelAfterGate : secretTunnel);
                break;
        }

        actionIndex++;
    }

    /* ===================== PATH BUILDING ===================== */
    private void buildPaths() {
        spike1 = buildSimplePickup(moveToSpike1Pickup, moveToShootSpike1, 180);
        spike2 = buildSimplePickup(moveToSpike2Pickup, moveToShootSpike2, 180);
        gatePickup = buildSimplePickup(moveToGate, moveToShootGate, 180);
        gateNoIntake = buildSimplePickup(moveToGateNoIntake, moveToShootGateNoIntake, 270);

        humanPlayer = follower.pathBuilder()
                .addPath(preHumanPlayer)
                .addPath(moveToHumanPlayer)
                .addParametricCallback(0.1, () -> intakeSystem.getTurnOnAction(true))
                .addPath(moveToShootHumanPlayer)
                .addParametricCallback(0.1, ()->intakeSystem.getTurnOffAction())
                .addParametricCallback(0.9, () -> launchSystem.getPerformLaunchOnAllSlots())
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .build();

        secretTunnel = buildSimplePickup(SecretTunnel, postSecretTunnel, 180);
        tunnelAfterGate = buildSimplePickup(SecretTunnelAfterGate, postSecretTunnel, 180);

        preloadShot = follower.pathBuilder()
                .addPath(shootPreload)
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .addParametricCallback(0.1, ()->intakeSystem.getTurnOffAction())
                .addParametricCallback(0.5, () -> launchSystem.getPerformLaunchOnAllSlots())
                .build();
    }

    private PathChain buildSimplePickup(Path in, Path out, double heading) {
        return follower.pathBuilder()
                .addPath(in)
                .addParametricCallback(0.1, () -> intakeSystem.getTurnOnAction(true))
                .addPath(out)
                .addParametricCallback(0.1, ()->intakeSystem.getTurnOffAction())
                .addParametricCallback(0.9, () -> stopShootAndHold())
                .setConstantHeadingInterpolation(Math.toRadians(heading))
                .build();
    }

    /* ===================== WAIT LOGIC ===================== */
    private void pauseAndWait() {
        follower.pausePathFollowing();
        waitTimer.reset();
        waiting = true;
    }

    /* ===================== HELPERS ===================== */
    private void enforcePreloadFirst() {
        actionQueue.remove(AutoAction.SHOOTING_PRELOADS);
        actionQueue.add(0, AutoAction.SHOOTING_PRELOADS);
    }

    private void moveMoveOffLineToEnd() {
        if (actionQueue.contains(AutoAction.MOVE_OFF_LINE)) {
            actionQueue.remove(AutoAction.MOVE_OFF_LINE);
            actionQueue.add(AutoAction.MOVE_OFF_LINE);
        }
    }

    private boolean isSecretTunnelAfterGate() {
        int tunnel = actionQueue.indexOf(AutoAction.SECRET_TUNNEL);
        if (tunnel <= 0) return false;
        AutoAction prev = actionQueue.get(tunnel - 1);
        return prev == AutoAction.GATE_PICKUP || prev == AutoAction.GATE_NO_INTAKE;
    }

    private void sendTelemetry() {
        telemetry.addData("Queue", actionQueue);
        if (!actionQueue.isEmpty()) {
            AutoAction last = actionQueue.get(actionQueue.size() - 1);
            telemetry.addData("Wait for last added", actionWaitTimes.getOrDefault(last, 0.0) + "s");
        }
        telemetry.update();
    }

    private void stopShootAndHold() {
        follower.pausePathFollowing();
        launchSystem.getPerformLaunchOnAllSlots();
        waitTimer.reset();
        waiting = true;
        waitSeconds = SHOOT_HOLD_SECONDS;
    }
}