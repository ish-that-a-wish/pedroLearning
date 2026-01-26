package org.firstinspires.ftc.teamcode.Tests;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@Disabled
@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Config // Panels
public class AutoTest extends OpMode {

    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState = 0; // Current autonomous path state (state machine)
//    private Paths paths; // Paths defined in the Paths class
    public PathChain Path1;
    public double Wait2;
    public PathChain Path3;
    public double Wait4;
    public PathChain Path5;
    public PathChain Path6;

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(56, 10, Math.toRadians(90)));

        makePaths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine
        runPaths();
        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void makePaths(Follower follower) {
        Path1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 10.000), new Pose(56.000, 28.000))
                )
                .setTangentHeadingInterpolation()
                .build();
        Path3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(56.000, 28.000), new Pose(38.000, 28.000))
                )
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
        Path5 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(38.000, 28.000),
                                new Pose(20.204, 38.571),
                                new Pose(38.000, 56.000)
                        )
                )
                .setTangentHeadingInterpolation()
                .build();

        Path6 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(new Pose(38.000, 56.000), new Pose(38.000, 75.000))
                )
                .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(0))
                .build();
    }
    public void runPaths(){
        switch(pathState){
            case 0:
                follower.followPath(Path1);
                pathState += 1;
                Log.i("MOVING FORWARD", "CURRENT POSE: " + follower.getPose());
                break;
            case 1:
                if(!follower.isBusy()){
                    follower.followPath(Path3);
                    Log.i("STRAFING", "CURRENT POSE: " + follower.getPose());
                    pathState += 1;
                    break;
                }
            case 2:
                if(!follower.isBusy()){
                    follower.followPath(Path5);
                    Log.i("SPLINING", "CURRENT POSE: " + follower.getPose());
                    pathState += 1;
                    break;
                }
            case 3:
                if(!follower.isBusy()){
                    follower.followPath(Path6);
                    Log.i("STRAFING", "CURRENT POSE: " + follower.getPose());
                    pathState += 1;
                    break;
                }
        }
    }
    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}