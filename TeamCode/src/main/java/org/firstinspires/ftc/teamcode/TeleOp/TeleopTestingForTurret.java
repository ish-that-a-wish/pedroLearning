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
import com.qualcomm.hardware.lynx.LynxController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LauncherSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSort;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

@Config
@TeleOp
public class TeleopTestingForTurret extends LinearOpMode {
    public ChassisControl chassis;
    private Follower follower;
    private RobotHardware robotHardware;
    private IntakeSubsystem intake;
    private LauncherSubsystem launch;
    private SpindexSort spindexSort;
    @Override
    public void runOpMode() throws InterruptedException {
            robotHardware = new RobotHardware(this.hardwareMap);
            intake = new IntakeSubsystem(this.hardwareMap);
            spindexSort = new SpindexSort(telemetry, robotHardware);
            launch = new LauncherSubsystem(robotHardware, follower, this.hardwareMap, spindexSort);

            follower = Constants.createFollower(this.hardwareMap);
            follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
            chassis = new ChassisControl(gamepad1, follower, this.hardwareMap);

            chassis.init();
            spindexSort.init();

            waitForStart();

            while(opModeIsActive()){
                chassis.update();
                launch.update();
                spindexSort.update();


                if(gamepad1.a) {launch.shootColor();}
                CommandScheduler.getInstance().run();
                telemetry.update();
        }
    }
}
