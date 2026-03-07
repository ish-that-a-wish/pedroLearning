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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

@Config
@TeleOp
public class TeleopTestingForTurret extends LinearOpMode {
    public static boolean shootOnMove = false; // set by false by defualt
    public ChassisControl chassis;
    private Follower follower;
    private RobotHardware robotHardware;
    private SpindexSubsystem spindex;
    private IntakeSubsystem intake;
    private LauncherSubsystem launch;
    @Override
    public void runOpMode() throws InterruptedException {
            robotHardware = new RobotHardware(this.hardwareMap);
            spindex = new SpindexSubsystem(this.robotHardware);
            intake = new IntakeSubsystem(this.hardwareMap);

            follower = Constants.createFollower(this.hardwareMap);
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            chassis = new ChassisControl(gamepad1, follower, this.hardwareMap);

            launch = new LauncherSubsystem(robotHardware, follower, this.hardwareMap, spindex);

            chassis.init();
            spindex.initMove();

            waitForStart();

            while(opModeIsActive()){
                chassis.update();
                launch.update();
                CommandScheduler.getInstance().run();

                //still have sum goof spindex logic to fix
                if(!spindex.isReadyToLaunch()){intake.runIntake(); spindex.intakeBalls();}
                if(gamepad1.a) CommandScheduler.getInstance().schedule(launch.shootAll());

                launch.setShootOnMove(shootOnMove); //constantly update shoot on move for testing

                telemetry.update();
        }
    }
}
