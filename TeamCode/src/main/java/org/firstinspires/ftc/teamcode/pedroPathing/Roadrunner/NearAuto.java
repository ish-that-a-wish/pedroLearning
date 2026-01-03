package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

import java.nio.file.Paths;

@Autonomous (name = "Near Auto", group = "LinearOpMode")
public class NearAuto extends LinearOpMode {
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    public static Pose StartPose = new Pose(22.000, 125.000);
    public static Pose EndPoseBase = new Pose(30.000, 115.000);
    public static Pose ControlPoint1A = new Pose(51.000, 71.000);
    public static Pose ControlPoint1B = new Pose(50.000, 88.000);
    public static Pose EndPose1 = new Pose(30.000, 84.000);
    public static Pose ControlPoint2A = new Pose(60.000, 54.000);
    public static Pose ControlPoint2B = new Pose(65.000, 64.000);
    public static Pose EndPose2 = new Pose(30.000, 60.000);
    public static Pose ControlPoint3A = new Pose(41.000, 24.000);
    public static Pose ControlPoint3B = new Pose(50.000, 38.000);
    public static Pose EndPose3 = new Pose(30.000, 35.000);
    public static Pose ShootPose = new Pose(61.000, 84.000);
    public PathChain Base;
    public PathChain Path1;
    public PathChain Path2;
    public PathChain Path3;

    private int clicks = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        while (opModeInInit()) {
            telemetry.addData("Clicks: ", clicks);
            telemetry.update();
            follower = Constants.createFollower(hardwareMap);
            follower.setStartingPose(new Pose(22, 125, Math.toRadians(144)));
            makePaths(follower);

            if (gamepad1.dpadUpWasReleased()) {
                clicks += 1;
            }

        }

        waitForStart();
        while (opModeIsActive()) {
            follower.update();
            runPaths();
            telemetry.addData("PathState", pathState);
            telemetry.update();
        }

    }

    public void makePaths(Follower follower) {
        Base = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(
                                StartPose,
                                EndPoseBase
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(144))
                .build();
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                EndPoseBase,
                                ControlPoint1A,
                                ControlPoint1B,
                                EndPose1
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                EndPose1,
                                ShootPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
        Path2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootPose,
                                ControlPoint2A,
                                ControlPoint2B,
                                EndPose2
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                EndPose2,
                                ShootPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))

                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                ShootPose,
                                ControlPoint3A,
                                ControlPoint3B,
                                EndPose3
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))
                .addPath(
                        new BezierLine(
                                EndPose3,
                                ShootPose
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(144), Math.toRadians(180))
                .build();
    }

    public void runPaths() {
        switch (pathState) {
            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(Base);
                    pathState += 1;
                }
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(Path1);
                    pathState += 1;
                }
                break;

            case 2:
                if (clicks <= pathState) {
                    if (!follower.isBusy()) {
                        follower.followPath(Path2);
                        pathState += 1;
                    }
                }
                break;
            case 3:
                if (clicks <= pathState) {
                    if (!follower.isBusy()) {
                        follower.followPath(Path3);
                        pathState += 1;
                    }
                }
                break;

        }
    }
}
