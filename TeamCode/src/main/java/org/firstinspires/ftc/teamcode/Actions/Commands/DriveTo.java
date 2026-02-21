package org.firstinspires.ftc.teamcode.Actions.Commands;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

@Config
public class DriveTo extends CommandBase {
    public static double xTol = 0.1;
    public static double yTol = 0.1;
//
//    private Pose targetPose;
//    private final Follower follower;
//
//    private PathChain move;
//    private boolean pathScheduled = false;
//
//    // Optional jiggle
//    private boolean jiggleForward = false;
//    private boolean startJiggling = false;
//    private final int jiggleAmount = 5;
//    public static int xBuffer = 3;
//    public static int yBuffer = 3;
//
//    public DriveTo(Pose targetPose, Follower follower) {
//        this.targetPose = targetPose;
//        this.follower = follower;
//    }
//
//    @Override
//    public void initialize() {
//        pathScheduled = false;
//        jiggleForward = false;
//        startJiggling = false;
//        buildPath();
//        Log.i("DriveTo", "Initialized");
//    }
//
//    @Override
//    public void execute() {
//        Log.i("DriveTo ", "Follower current pose: " + follower.getPose());
//        Log.i("DriveTo ", "X error" + String.valueOf(Math.abs(targetPose.getX() - follower.getPose().getX())));
//        Log.i("DriveTo ", "Y error" + String.valueOf(Math.abs(targetPose.getY() - follower.getPose().getY())));
//
//        if (!pathScheduled) {
//            follower.followPath(move);
//            pathScheduled = true;
//        }
////        if(targetPose.getX() - follower.getPose().getX() < pathBuffer && targetPose.getPose().getY() - follower.getPose().getY() < pathBuffer){
////            Log.i("DriveTo ", "pausing path following");
////            follower.pausePathFollowing();
////        }
//
//        // Optional jiggle logic (commented until needed)
//        /*
//        if (!follower.isBusy() && !startJiggling) {
//            startJiggling = true;
//            jiggleForward = !jiggleForward;
//            jiggle();
//        }
//        */
//
//        Log.i("DriveTo", "Running");
//    }
//
//    @Override
//    public boolean isFinished() {
//        Log.i("DriveTo ", "Finished");
//        return false;
////        if(targetPose.getX() - follower.getPose().getX() < pathBuffer && targetPose.getPose().getY() - follower.getPose().getY() < pathBuffer){
////            Log.i("DriveTo ", "done running");
////            return true;
////        }
////        else{
////            Log.i("DriveTo ", "Still running");
////
////            return false;
////        }
////        if(spindex.isFull()){
////            Log.i("DriveTo ", "Command ending");
////        }
////        return spindex.isFull();
//    }
//
//    @Override
//    public void end(boolean interrupted) {
//
//        Log.i("DriveTo", interrupted ? "Interrupted" : "Ended");
//
//    }
//
//    private void buildPath() {
//
//        // ALWAYS use live pose
//        Pose currentPose = follower.getPose();
//
//        move = follower.pathBuilder()
//                .addPath(new BezierLine(currentPose, targetPose))
//                .setConstantHeadingInterpolation(Math.toRadians(90))
//                .build();
//    }
//
//    private void jiggle() {
//
//        Pose currentPose = follower.getPose();
//
//        double offset = jiggleForward ? jiggleAmount : -jiggleAmount;
//
//        Pose jigglePose = new Pose(
//                targetPose.getX(),
//                targetPose.getY() + offset,
//                targetPose.getHeading()
//        );
//
//        move = follower.pathBuilder()
//                .addPath(new BezierLine(currentPose, jigglePose))
//                .setConstantHeadingInterpolation(currentPose.getHeading())
//                .build();
//
//        follower.followPath(move);
//    }

        private final Pose targetPose;
        private final Follower follower;

        private PathChain move;
        private boolean pathScheduled = false;

        // Optional jiggle
        private boolean jiggleForward = false;
        private boolean startJiggling = false;
        private final int jiggleAmount = 5;
        private final int pathBuffer = 3;
        private SpindexSubsystem spindex;

        public DriveTo(Pose targetPose, Follower follower, SpindexSubsystem spindex) {
            this.spindex = spindex;
            this.targetPose = targetPose;
            this.follower = follower;
        }

        @Override
        public void initialize() {
            follower.resumePathFollowing(); // just in case following is paused
            pathScheduled = false;
            jiggleForward = false;
            startJiggling = false;

            buildPath();

//            Log.i("DriveTo", "Initialized");
        }

        @Override
        public void execute() {
//            Log.i("DriveTo ", "Follower current pose: " + follower.getPose());
            if (!pathScheduled) {
                follower.followPath(move);
                pathScheduled = true;
            }

//        if(targetPose.getX() - follower.getPose().getX() < pathBuffer && targetPose.getPose().getY() - follower.getPose().getY() < pathBuffer){
//            Log.i("DriveTo ", "pausing path following");
//            follower.pausePathFollowing();
//        }

            // Optional jiggle logic (commented until needed)

        if (!follower.isBusy() && !startJiggling) {
            startJiggling = true;
            jiggleForward = !jiggleForward;
            jiggle();
        }

//            Log.i("DriveTo", "Running");
        }

        @Override
        public boolean isFinished() {
//            if(targetPose.getX() - follower.getPose().getX() < pathBuffer && targetPose.getPose().getY() - follower.getPose().getY() < pathBuffer){
//                Log.i("DriveTo ", "done running");
//                return true;
//            }
//            else{
//                Log.i("DriveTo ", "Still running");
//                return false;
//            }
//            Log.i("Is DriveTo Finished ", String.valueOf(follower.atPose(targetPose, xTol, yTol)));
            return follower.atPose(targetPose, xTol, yTol);
        }

    @Override
    public void end(boolean interrupted) {
//        follower.pausePathFollowing();
        Log.i("DriveTo", interrupted ? "Interrupted" : "Ended");
    }

        private void buildPath() {

            // ALWAYS use live pose
            Pose currentPose = follower.getPose();

            move = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, targetPose))
                    .setConstantHeadingInterpolation(Math.toRadians(90))
                    .build();
        }

        private void jiggle() {
//            Log.i("DriveTo ", "Jiggling");
            Pose currentPose = follower.getPose();

            double offset = jiggleForward ? jiggleAmount : -jiggleAmount;

            Pose jigglePose = new Pose(
                    targetPose.getX(),
                    targetPose.getY() + offset,
                    targetPose.getHeading()
            );

            move = follower.pathBuilder()
                    .addPath(new BezierLine(currentPose, jigglePose))
                    .setConstantHeadingInterpolation(currentPose.getHeading())
                    .build();

            follower.followPath(move);
        }
    }

