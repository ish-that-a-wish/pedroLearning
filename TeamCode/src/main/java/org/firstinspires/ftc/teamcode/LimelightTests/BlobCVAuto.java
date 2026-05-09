package org.firstinspires.ftc.teamcode.LimelightTests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Config
@Autonomous
public class BlobCVAuto extends LinearOpMode {

    public Follower follower;
    private RobotHardware robo;
    public Limelight3A limelight;

    private IntakeSubsystem intake;
    private SpindexSubsystem spindex;

    // Tunable offsets via FTC Dashboard
    public static double yOffset = 0;
    public static double xOffset = 0;
    public static double initHeading = 90;

    // Computed global target
    private double targetX;
    private double targetY;

    // Paths
    public PathChain xPath;
    public PathChain yPath;

    // State machine
    private int pathState = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        double currentHeadingRad = Math.toRadians(initHeading);

        // Hardware init
        robo = new RobotHardware(this.hardwareMap);
        intake = new IntakeSubsystem(this.hardwareMap);
        spindex = new SpindexSubsystem(robo);
        spindex.initMove();

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        follower = Constants.createFollower(hardwareMap);

        // Init loop
        while (opModeInInit()) {
            limelight.setPollRateHz(11);
            limelight.pipelineSwitch(1);
            limelight.start();
            follower.setStartingPose(new Pose(72, 72, currentHeadingRad));
        }

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            switch (pathState) {

                case 0: // Detect blob and build paths
                    LLResult llResult = limelight.getLatestResult();

                    if (llResult == null) {
                        telemetry.addLine("Waiting for Limelight result...");
                        telemetry.update();
                        break;
                    }

                    double[] blobDist = llResult.getPythonOutput();
                    if (blobDist == null || blobDist.length < 4) {
                        telemetry.addLine("Invalid blob data, retrying...");
                        telemetry.update();
                        break;
                    }

                    // localX = camera lateral (strafe), localY = camera forward
                    double localX = blobDist[2];
                    double localY = blobDist[3];

                    Pose robotPose = follower.getPose();
                    double heading = robotPose.getHeading();

                    // FIX: localY is forward (drives global position along heading),
                    // localX is lateral (drives global position perpendicular to heading)
                    targetX = robotPose.getX() + (localY * Math.cos(heading) - localX * Math.sin(heading));
                    targetY = robotPose.getY() + (localY * Math.sin(heading) + localX * Math.cos(heading));

                    Log.i("BlobCV", "localX=" + localX + " localY=" + localY
                            + " targetX=" + targetX + " targetY=" + targetY);

                    buildPaths(robotPose, heading);
                    follower.followPath(xPath);
                    pathState = 1;
                    break;

                case 1: // Moving along X leg
                    if (!follower.isBusy()) {
                        follower.followPath(yPath);
                        pathState = 2;
                    }
                    break;

                case 2: // Moving along Y leg
                    if (!follower.isBusy()) {
                        pathState = 3;
                        Log.i("BlobCV", "Arrived at target.");
                    }
                    break;

                case 3: // Done — idle
                    telemetry.addLine("Arrived at target.");
                    telemetry.update();
                    break;
            }

            intake.runIntake();
            spindex.intakeBalls();

            if (spindex.isReadyToLaunch()) {
                follower.breakFollowing();
            }

            telemetry.addData("Path State", pathState);
            telemetry.addData("Target X", targetX);
            telemetry.addData("Target Y", targetY);
            telemetry.update();
        }
    }

    public void buildPaths(Pose robotPose, double headingRad) {
        // X leg: move from current pose to (targetX, currentY)
        xPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        new Pose(robotPose.getX(), robotPose.getY()),
                        new Pose(targetX + xOffset, robotPose.getY())
                )))
                .setConstantHeadingInterpolation(headingRad)
                .build();

        // Y leg: move from (targetX, currentY) to (targetX, targetY)
        yPath = follower.pathBuilder()
                .addPath(new Path(new BezierLine(
                        new Pose(targetX + xOffset, robotPose.getY()),
                        new Pose(targetX + xOffset, targetY + yOffset)
                )))
                .setConstantHeadingInterpolation(headingRad)
                .build();
    }
}