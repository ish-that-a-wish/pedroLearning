package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

import java.util.ArrayList;

@Autonomous
public class NearAutoEnums extends LinearOpMode {

    /* ---------------- ENUM ---------------- */

    public enum AutoAction {
        PRELOADS,
        FIRST_SPIKE,
        SECOND_SPIKE,
        GATE_PICKUP,

        GATE_SHOOTING

    }

    /* ---------------- FIELDS ---------------- */

    private ArrayList<AutoAction> orderToPickup = new ArrayList<>();
    private int actionIndex = 0;

    private Follower follower;

    /* ---------------- POSES ---------------- */

    public Pose INIT_POSE = new Pose(22, 125, Math.toRadians(180));
    public static Pose PRELOADS = new Pose(30, 115, Math.toRadians(180));
    public static Pose FIRST_SPIKE = new Pose(30, 84, Math.toRadians(180));
    public static Pose SECOND_SPIKE = new Pose(30, 60, Math.toRadians(180));

    public static Pose SHOOTING_POSE = new Pose(61, 84, Math.toRadians(180));
    public static Pose GATE_PICKUP = new Pose(14, 63, Math.toRadians(135));
    public static Pose SHOOTING_AFTER_GATE = new Pose(58, 78, Math.toRadians(270));

    private Pose CONTROL_POINT_SECOND_SPIKE = new Pose(43, 58);

    /* ---------------- PATHS ---------------- */

    private PathChain pickUpSpike1;
    private PathChain pickUpSpike2;
    private PathChain gatePickup;

    private PathChain gateToShooting;
    private PathChain preloads;



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

            telemetry.addLine("CLICK ORDER QUEUE:");
            telemetry.addData("Queue", orderToPickup.toString());
            telemetry.update();
        }

        waitForStart();

        /* -------- AUTON LOOP -------- */

        while (opModeIsActive()) {
            follower.update();
            runPaths();

            telemetry.addData("Current Action Index", actionIndex);
            telemetry.addData("Queue Size", orderToPickup.size());
            telemetry.update();
        }
    }

    /* ---------------- PATH BUILDING ---------------- */

    private void buildPaths() {
        preloads = follower.pathBuilder()
                .addPath(new BezierLine(INIT_POSE, PRELOADS))
                .setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();

        pickUpSpike1 = follower.pathBuilder()
                .addPath(new BezierLine(PRELOADS, FIRST_SPIKE))
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(new BezierLine(FIRST_SPIKE, SHOOTING_POSE))
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
                .build();
//                .addPath(new BezierLine(GATE_PICKUP, SHOOTING_AFTER_GATE))
//                .setLinearHeadingInterpolation(
//                        GATE_PICKUP.getHeading(),
//                        SHOOTING_AFTER_GATE.getHeading()
//                )
//                .build();

        gateToShooting= follower.pathBuilder()
                .addPath(new BezierLine(GATE_PICKUP, SHOOTING_AFTER_GATE))
                .setLinearHeadingInterpolation(
                        GATE_PICKUP.getHeading(),
                        SHOOTING_AFTER_GATE.getHeading()
                )
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

        AutoAction currentAction = orderToPickup.get(actionIndex);

        switch (currentAction) {
            case PRELOADS:
                follower.followPath(preloads);
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

            case GATE_SHOOTING:
                follower.followPath(gateToShooting);
                break;

        }

        actionIndex++; // Advance to next queued action
    }
}
