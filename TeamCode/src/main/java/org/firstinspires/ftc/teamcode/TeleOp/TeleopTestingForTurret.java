package org.firstinspires.ftc.teamcode.TeleOp;

import static org.firstinspires.ftc.teamcode.AutoWithRR.FirstSpike.kickerDelay;
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Tests.Constants;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

import java.util.List;

@Config
@TeleOp
public class TeleopTestingForTurret extends LinearOpMode {
    //MIDPOINT: (128,135)
    //FARPOINT: (133, 137)
    
    public static double xTarget = 133;
    public static double yTarget = 137;
    public static int lastKickWait = 130;
    public static int topWait = 150;
    public static int midWait = 150;
    public static int bottomWait = 100;
    public static double power = 0.2;
    public static int velocity = 2000;
    private final double turretTicksPerRev = 751.8 * 5; // for torque increase
    private Pose RED_GOAL_POSE = new Pose(xTarget, yTarget, Math.toRadians(45)); // pose of the red goal
    private Pose pose;
    private Follower follower;
    private DcMotorEx turret;
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private RobotHardware robotHardware;
    private double distFromGoal;
    private SpindexSubsystem spindex;
    private IntakeSubsystem intake;
    @Override
    public void runOpMode() throws InterruptedException {
        robotHardware = new RobotHardware(this.hardwareMap);
        spindex = new SpindexSubsystem(this.robotHardware);
        intake = new IntakeSubsystem(this.hardwareMap);

        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        follower.drivetrain.startTeleopDrive(true);

        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPower(power);

        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        //this motor is oriented backwards, hence reversing direction
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        //this motor is oriented backwards, hence reversing direction
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        spindex.initMove();
        waitForStart();

        while(opModeIsActive()){
            RED_GOAL_POSE = new Pose(xTarget, yTarget);

            double speed = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(speed) + Math.abs(strafe) + Math.abs(turn), 1);

            double frontLeftPower = (speed + turn + strafe) / denominator;
            double frontRightPower = (speed - turn - strafe) / denominator;
            double backLeftPower = (speed + turn - strafe) / denominator;
            double backRightPower = (speed - turn + strafe) / denominator;

            frontLeftDriveMotor.setPower(frontLeftPower);
            frontRightDriveMotor.setPower(frontRightPower);
            backLeftDriveMotor.setPower(backLeftPower);
            backRightDriveMotor.setPower(backRightPower);

            follower.update();
            pose = follower.getPose();

            distFromGoal = follower.getPose().distanceFrom(RED_GOAL_POSE);
            keepLauncherWarm(distFromGoal); // tell the dist from goal
            updateTurret();
            telemetry.update();

            if(!spindex.isReadyToLaunch()) {
                intake();
            }
            if(gamepad1.dpad_down){
                List<Double> visorPoses = getVisorPoses(follower.getPose().distanceFrom(RED_GOAL_POSE));
                CommandScheduler.getInstance().schedule(
                        new SequentialCommandGroup(
                            new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_1)),
                            new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(0))),
                            shoot(false),
                            new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_3)),
                            new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(1))),
                            shoot(false),
                            new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_2)),
                            new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(1))),
                            shoot(true),
                            new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
                        )
                );
            }

            CommandScheduler.getInstance().run();

            Log.i("Launcher: ", "Dist from goal: " + distFromGoal);
            Log.i("Turret: ","Amount to move to goal: " +  calculateAmountToTurnToGoal());
            Log.i("Turret: ", "Positional alignment pos: " + pose);
            Log.i("Turret: ", "Degrees to ticks: " + degreesToTicks(calculateAmountToTurnToGoal()));
    }
}
    public double calculateAmountToTurnToGoal(){
        double amountToTurn = Math.atan2(RED_GOAL_POSE.getY() - pose.getY(), RED_GOAL_POSE.getX() - pose.getX());
        return Math.toDegrees(amountToTurn - pose.getHeading());
    }
    public double degreesToTicks(double deg){
        return (deg/360) * turretTicksPerRev;
    }
    public void updateTurret(){
        double deg = calculateAmountToTurnToGoal();
        int ticks = (int) degreesToTicks(deg);

        turret.setDirection(ticks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        Log.i("Turret: ", "Current Direction: " + turret.getDirection());

        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(velocity);
        follower.getVelocity();
    }
    public void keepLauncherWarm(double distance){
         BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distance);

         double flywheelSpeed = ballLaunchParameters.flywheelVelocity;

         robotHardware.setFlywheelVelocityInTPS(flywheelSpeed); // set the speed of the flywheel

         Log.i("Launcher: "," Target Velocity: " + flywheelSpeed);
    }
    public void intake(){
        intake.runIntake();
        spindex.intakeBalls();
    }

    public List<Double> getVisorPoses(double distFromGoal){
        BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distFromGoal);
        Log.i("Launcher: ", "Visor poses: " + ballLaunchParameters.visorPositions);
        return ballLaunchParameters.visorPositions;
    }

    public Command shoot(boolean lastWait){
        return new SequentialCommandGroup(
                new WaitCommand(topWait),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING)),
                new WaitCommand(lastWait ? lastKickWait : midWait),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(bottomWait)
        );
    }
}
