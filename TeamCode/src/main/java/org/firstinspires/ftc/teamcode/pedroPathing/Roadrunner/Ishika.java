//package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;
//
//import com.acmerobotics.roadrunner.Pose2d;
//import com.pedropathing.paths.Path;
//import com.pedropathing.paths.PathBuilder;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
////import com.acmerobotics.roadrunner.geometry.Pos
//
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.BezierLine;
//
//import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;
//
//@Autonomous(name="RR+Pedro Straight Line", group="Hybrid")
//public class Ishika extends LinearOpMode {
//
//    private SampleMecanumDrive rrDrive;
//    private Follower pedroFollower;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        // Initialize RoadRunner
//        rrDrive = new DriveCommandMessage(hardwareMap);
//
//        // Initialize PedroPathing follower
//        pedroFollower = Constants.createFollower(hardwareMap);
//
//        waitForStart();
//        if (isStopRequested()) return;
//
//        // Start pose at origin
//        Pose2d startPose = new Pose2d(0, 0, 0);
//        rrDrive.setPoseEstimate(startPose);
//
//        // RoadRunner trajectory: straight 12 inches forward
//        TrajectorySequence traj = rrDrive.trajectorySequenceBuilder(startPose)
//                .forward(12)  // short straight line
//                .build();
//
//        // Pedro path: straight line from start to 12 inches forward
//        PathBuilder pb = pedroFollower.pathBuilder();
//        pb.addPath(new BezierLine(startPose, new Pose2d(12, 0, 0)))
//                .setLinearHeadingInterpolation(startPose.getHeading(), 0);
//        Path straightPedroPath = pb.build();
//        pedroFollower.followPath(straightPedroPath);
//
//        // Start RoadRunner async following
//        rrDrive.followTrajectorySequenceAsync(traj);
//
//        // Hybrid loop
//        while (opModeIsActive() && !isStopRequested()) {
//
//            // RoadRunner internal update
//            rrDrive.update();
//
//            // Current robot pose
//            Pose2d currentPose = rrDrive.getPoseEstimate();
//
//            // RR target pose (next waypoint / last pose)
//            Pose2d targetPose = rrDrive.getLastPose();
//
//            // Pedro correction
//            pedroFollower.update(currentPose, targetPose);
//
//            // Apply corrected outputs
//            pedroFollower.applyOutput(rrDrive);
//
//            // Telemetry
//            telemetry.addData("RR pose", currentPose.toString());
//            telemetry.addData("Pedro error", pedroFollower.getError().toString());
//            telemetry.update();
//        }
//    }
//}
//
