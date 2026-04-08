package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.LAUNCH_KICK_KICKING;
import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.LAUNCH_KICK_RESTING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.INTAKE_POSE_1;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_1;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_2;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_3;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSort;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.sql.Time;
import java.util.Calendar;
import java.util.Date;
import java.util.List;
import java.util.Timer;

@Config
@TeleOp
public class TeleopTestingForTurret extends LinearOpMode {
    public ChassisControl chassis;
    private Follower follower;
    private RobotHardware robotHardware;
    private SpindexSubsystem spindex;
    private IntakeSubsystem intake;
    private LauncherSubsystem launch;
    private double Voltage = 12; // defualt to max

    List<LynxModule> hubs;
//    private SpindexSort spindexSort;
    @Override
    public void runOpMode() throws InterruptedException {
            robotHardware = new RobotHardware(this.hardwareMap);
            spindex = new SpindexSubsystem(this.robotHardware);
            intake = new IntakeSubsystem(this.hardwareMap);
//            spindexSort = new SpindexSort(telemetry, robotHardware);
            intake.setPower(1);
            follower = Constants.createFollower(this.hardwareMap);
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            chassis = new ChassisControl(gamepad1, follower, this.hardwareMap);

            launch = new LauncherSubsystem(robotHardware, follower, this.hardwareMap, spindex);

            chassis.init();
//            spindexSort.init();
            spindex.initMove();
            waitForStart();

            while(opModeIsActive()){
                chassis.update();
                launch.update();
//                spindexSort.update();

                //still have sum goof spindex logic to fix
                if(!spindex.isReadyToLaunch()){
                    intake.runIntake();
                    spindex.intakeBalls();
                }
                if(spindex.isReadyToLaunch()){intake.stopIntake();}

                if(gamepad1.aWasPressed()) {
                    CommandScheduler.getInstance().schedule(launch.shootAll());
                }

                if(gamepad1.bWasPressed()){
                    follower.setPose(new Pose(72,72, Math.toRadians(90)));
                }

                telemetry.addData("SPINDEX", "SPINDEX READY TO LAUNCH: " + spindex.isReadyToLaunch());
                telemetry.addData("SPINDEX: ", "CURRENT POSE: " + spindex.getCurrentPose().toString());
                telemetry.addData("LAUNCHER: ", "Flywheel speed: " + robotHardware.getFlywheelVelocityInTPS());

                CommandScheduler.getInstance().run();
                telemetry.update();

        }
    }
}
