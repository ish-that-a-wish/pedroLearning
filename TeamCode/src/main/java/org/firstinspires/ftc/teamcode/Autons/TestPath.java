package org.firstinspires.ftc.teamcode.Autons;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.pedropathing.paths.callbacks.PathCallback;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Autonomous
public class TestPath extends LinearOpMode {
    public static Pose SHOOTPOSE = new Pose(60, 8, Math.toRadians(180));
    public static Pose InitPose = new Pose(56, 8, Math.toRadians(90));
    public static Pose control1 = new Pose(50,40);
    public static Pose PICKUP1 = new Pose(10, 36, Math.toRadians(180));
    public static Pose STARTINTAKE = new Pose(45, 29);
    public PathChain firstSpike;
    public Path intakeFirstSpike = new Path(new BezierCurve(InitPose, control1, PICKUP1)); // path to pickup
    public Path shootFirstSpike = new Path(new BezierLine(PICKUP1, SHOOTPOSE)); // path to shootFirst
    public Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower.setMaxPower(0.8);//sets the max power until you change it
        waitForStart();
        while(opModeIsActive()){
            
            //follower.followPath(pathChain1, 0.8, true); // hold end makes the follower hold the last point on the chain, maxPower is the maxpower outputted
        }
    }
    public void buildPaths(){
//        pathChain1 = follower.pathBuilder()
//                .addParametricCallback(0.7, () -> runIntake()) // works by running an action after the robot has completed a certain percent of the path like in this case 70%
//                .addTemporalCallback(500, () -> runIntake()) // works by running an action after a certain amount of time like in this case 500 ms
//                .addPoseCallback(new Pose(5, 5), ()->runIntake(), 0.8) // works by running an action after the robot reaches a certain pose like in this case 5,5 the t value is a guess at what percent of the path the robot has gone thru when it reaches the point
        firstSpike = follower.pathBuilder()
                .addPath(intakeFirstSpike)
                .setTangentHeadingInterpolation()
                .addPoseCallback(STARTINTAKE, ()->runIntake(), 0.5)
                .addPath(shootFirstSpike)
                .setConstantHeadingInterpolation(180)
                // you can also set constraints to determine when a path is complete
                .setTimeoutConstraint(200) // in ms how long the follower has to be correct for
                .setVelocityConstraint(0.05) // final speed should be 0.05
                .setHeadingConstraint(Math.toRadians(2)) // 2 degrees of error allowed
                .setTValueConstraint(0.98) // how far along the path the pathchain must be
                .setTranslationalConstraint(0.5) // max amount of translational error
                //.addLoopedCallback() // runs every loop of the pathchain, every time this pathchain is active also is custom callback
                //.curveThrough() // used to automatically generate bezier curves, useless if using path generator
                .build();
    }
    //example that does nothing, just here so no errors
    public void runIntake(){}
}
