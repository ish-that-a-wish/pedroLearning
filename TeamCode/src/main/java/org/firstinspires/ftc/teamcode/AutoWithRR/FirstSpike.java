//package org.firstinspires.ftc.teamcode.AutoWithRR;
//
//import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToShootSpike1;
//import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.moveToSpike1Pickup;
//import static org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants.shootPreload;
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandScheduler;
//import com.arcrobotics.ftclib.command.ParallelCommandGroup;
//import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
//import com.arcrobotics.ftclib.command.ParallelRaceGroup;
//import com.arcrobotics.ftclib.command.SequentialCommandGroup;
//import com.pedropathing.follower.Follower;
//import com.pedropathing.geometry.Pose;
//import com.pedropathing.paths.PathChain;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Actions.Commands.DriveTo;
//import org.firstinspires.ftc.teamcode.Actions.Commands.KickCommand;
//import org.firstinspires.ftc.teamcode.Actions.Commands.intakeCommand;
////import org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot;
//import org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot;
//import org.firstinspires.ftc.teamcode.Tests.Constants;
//import org.firstinspires.ftc.teamcode.common.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;
//import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;
//
//@Autonomous
//public class FirstSpike extends LinearOpMode {
//    private Follower follower;
//    private RobotHardware robotHardware;
//    private intakeSubsystem intakeSubsystem;
//    private SpindexSubsystemReal spindex;
//    private KickerSubsystem kicker;
//    @Override
//    public void runOpMode() throws InterruptedException {
//        follower = Constants.createFollower(this.hardwareMap);
//        follower.setStartingPose(new Pose(0,0, Math.toRadians(90))); // set the val to blue near init
//
//
//        robotHardware = new RobotHardware(this.hardwareMap);
//        spindex = new SpindexSubsystemReal(this.robotHardware);
//        intakeSubsystem = new intakeSubsystem(this.hardwareMap);
//        kicker = new KickerSubsystem(this.robotHardware, this.spindex);
//
//        CommandScheduler.getInstance().schedule(
//                false,
//                new SequentialCommandGroup(
////                        new ParallelDeadlineGroup(
////                            new DriveTo(targetPose, follower, spindex),
////                            new moveToEmptySpindexSlot(spindex),
////                            new intakeCommand(intakeSubsystem, true)
////                        ),
////                        new moveToEmptySpindexSlot(spindex)
//
//                        // run spindex and drive to finish first to finish moves to the next cmd
//                        new ParallelDeadlineGroup(
//                            new DriveTo(new Pose(0, 15, Math.toRadians(90)), follower, spindex),
//                            new intakeCommand(intakeSubsystem, true),
//                            new moveToEmptySpindexSlot(spindex)
//                    )
////                    new KickCommand(this.kicker)
//                )
//        );
////
////        CommandScheduler.getInstance().schedule(
////                new ParallelDeadlineGroup(
////                        driveCommand,
////                        intakeCommand,
////                        emptySlot)
////        );
//
//        waitForStart();
//
//        while(opModeIsActive()){
//            CommandScheduler.getInstance().run();
//
//            follower.update();
//        }
//    }
//}

package org.firstinspires.ftc.teamcode.AutoWithRR;

import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_1;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_2;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_3;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Actions.Commands.DriveTo;
import org.firstinspires.ftc.teamcode.Actions.Commands.TurretCommand;
import org.firstinspires.ftc.teamcode.Actions.Commands.intakeCommand;
import org.firstinspires.ftc.teamcode.Actions.Commands.LaunchCommand;
import org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot;
import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.LimelightAprilTagHelper;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

@Config
@Autonomous
public class FirstSpike extends LinearOpMode {
//    public static int spindexDelay =500;
    public static int kickerDelay = 182; // wait time is 182
    private Follower follower;
    private RobotHardware robotHardware;
    private moveToEmptySpindexSlot emptySlot;
    private IntakeSubsystem intakeSubsystem;

    private intakeCommand intakeCommand;
    private SpindexSubsystem spindex;
    private DriveTo driveCommand;
    private KickerSubsystem kickerSubsystem;
    private LauncherSubsystem launchSystem;
    private LaunchCommand launchCommand;
    private LimelightAprilTagHelper aprilTagHelper;
    @Override
    public void runOpMode() throws InterruptedException {
        //driving
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(new Pose(72,72, Math.toRadians(90))); // set the val to blue near init
        robotHardware = new RobotHardware(this.hardwareMap);

        //intake subsystems
        spindex = new SpindexSubsystem(this.robotHardware);
        intakeSubsystem = new IntakeSubsystem(this.hardwareMap);
        kickerSubsystem = new KickerSubsystem(robotHardware);

        //launcher
        aprilTagHelper = new LimelightAprilTagHelper(this.robotHardware);
        launchSystem = new LauncherSubsystem(this.robotHardware, aprilTagHelper);
        launchCommand = new LaunchCommand(this.launchSystem, this.robotHardware);

        //commands
        driveCommand = new DriveTo(new Pose(72, 87, Math.toRadians(90)), follower, spindex);
        emptySlot = new moveToEmptySpindexSlot(spindex);
        intakeCommand = new intakeCommand(intakeSubsystem, true);

        CommandScheduler.getInstance().registerSubsystem(kickerSubsystem, intakeSubsystem, spindex);
        CommandScheduler.getInstance().schedule(
                new ParallelCommandGroup(
                    new SequentialCommandGroup(
                            new ParallelDeadlineGroup(
                                    emptySlot,
                                    driveCommand,
                                    launchCommand,
                                    intakeCommand
                            ),
                            launch3Balls()
    //                        new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
                    ),
                    new TurretCommand(this.follower, this.hardwareMap))
        );

        waitForStart();

        while(opModeIsActive()){
            CommandScheduler.getInstance().run();
            if(!driveCommand.isFinished()) {
                follower.update();
            }
//            if(driveCommand.isFinished()){
//                follower.pausePathFollowing();
//            }
        }
    }
    private SequentialCommandGroup launch3Balls(){
        // 1 3 2 is faster
        return new SequentialCommandGroup(

                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_1)),
                kickBall(spindex.convertSpindexPoseToDouble(LAUNCH_POSE_1)),
                //assume that at the end of moveToEmptySlot we move to launch 1
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_3)),
                kickBall(spindex.convertSpindexPoseToDouble(LAUNCH_POSE_3)),
//                new WaitCommand(spindexDelay),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_2)),
                kickBall(spindex.convertSpindexPoseToDouble(LAUNCH_POSE_2))
        );
    }
    private SequentialCommandGroup kickBall(double SpindexKickingPose){
        return new SequentialCommandGroup(
                new WaitCommand(kickerDelay),
                new InstantCommand(() -> kickerSubsystem.moveKickerUp(SpindexKickingPose)),
                new WaitCommand(kickerDelay),
                new InstantCommand(() -> kickerSubsystem.moveKickerDown(SpindexKickingPose)),
                new WaitCommand(kickerDelay)
        );
    }
}