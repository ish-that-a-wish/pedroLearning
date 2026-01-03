package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.CallBacks.WaitCallback;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;
import com.acmerobotics.roadrunner.Action.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.ArrayList;

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

    public ElapsedTime waitTimer = new ElapsedTime();
    public Boolean waiting = false;

    /* ---------------- OPMODE ---------------- */

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(INIT_POSE);

        buildPaths();

        /* -------- INIT BUTTON SELECTION -------- */

        while (opModeInInit()) {

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

            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderToPickup.toString());
            telemetry.update();
        }

        waitForStart();

        /* -------- AUTON LOOP -------- */

        while (opModeIsActive()) {
            if (waiting && waitTimer.seconds() >= 5) {
                follower.resumePathFollowing();
                waiting = false;
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
                .addPath(new BezierCurve(INIT_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(FIRST_SPIKE, SHOOTING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.9, () -> {
                    follower.pausePathFollowing();
                    waitTimer.reset();
                    waiting = true;
                })
                .addPath(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        pickUpSpike2 = follower.pathBuilder()
                .addPath(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(SECOND_SPIKE, SHOOTING_POSE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .build();

        gatePickup = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTING_POSE, GATE_PICKUP))
                .setLinearHeadingInterpolation(
                        SHOOTING_POSE.getHeading(),
                        GATE_PICKUP.getHeading()
                )
                .addPath(new BezierLine(GATE_PICKUP, SHOOTING_AFTER_GATE))
                .setLinearHeadingInterpolation(
                        GATE_PICKUP.getHeading(),
                        SHOOTING_AFTER_GATE.getHeading()
                )
                .build();

        humanPlayerPickup = follower.pathBuilder()
                .addPath(new BezierLine(SHOOTING_AFTER_GATE, PRE_HUMAN_PLAYER))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(new BezierLine(PRE_HUMAN_PLAYER, HUMAN_PLAYER_PICKUP))
                .setConstantHeadingInterpolation(Math.toRadians(270))
                .addPath(new BezierLine(HUMAN_PLAYER_PICKUP, SHOOTING_AFTER_GATE))
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
        if(waiting) {
            return;
        }
        if (waiting && waitTimer.seconds() >= 5) {
            follower.resumePathFollowing();
            follower.breakFollowing();   // 🔑 THIS LINE
            waiting = false;
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
}
