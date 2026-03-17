package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.LAUNCH_KICK_KICKING;
import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.LAUNCH_KICK_RESTING;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.INTAKE_POSE_1;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_1;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_2;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.LAUNCH_POSE_3;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;

import java.util.ArrayList;
import java.util.List;

@Config
public class LauncherSubsystem extends SubsystemBase {
    //Basic required subsys
    public RobotHardware robotHardware;
    private Follower follower;
    public SpindexSubsystem spindex;
    public static double xTarget = 131;
    public static double yTarget = 138;
    public static double turretLimit = 120;

    // Goal following vars
    private Pose RED_GOAL_POSE = new Pose(xTarget, yTarget, Math.toRadians(45)); // pose of the red goal;
    public Pose currentPose;
    public double distFromGoal;

    //turret specific vars
    private final double turretTicksPerRev = 751.8 * 5; // for torque increase
    private DcMotorEx turret;

    // PD + Feedforward vars
    public static double kP = 0.0025;
    public static double kD = 0.00045;
    public static double kF = 0.0009;
    public static double maxTurretPower = 0.85;

    private double lastError = 0;
    private double lastHeading = 0;
    private ElapsedTime pdTimer = new ElapsedTime();

    //SOTM
    private boolean shootOnMove = true;
    private Pose goalUsed;

    // current optimized shot times, fix later
    public static double shot1Ms = 350;
    public static double shot2Ms = 570;
    public static double shot3Ms = 470;
    public Pose futureGoal; // defualt to it being noting
    public static double power = 0.8;
    public static double velocity = 4000;
    private ElapsedTime timer = new ElapsedTime();

    public static boolean usePDF = false;
    public List<GameColors> motif = new ArrayList<>();
    public LauncherSubsystem(RobotHardware robotHardware, Follower follower, HardwareMap hardwareMap, SpindexSubsystem spindex){
        this.spindex = spindex;
        this.follower = follower;
        this.robotHardware = robotHardware;

        currentPose = follower.getPose();

        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPower(power);

        pdTimer.reset();
        lastHeading = currentPose.getHeading();


        // add motif scanning later
        motif.add(GameColors.PURPLE);
        motif.add(GameColors.GREEN);
        motif.add(GameColors.PURPLE);
    }

    public void update(){
        currentPose = follower.getPose();
        if(shootOnMove) shootOnMove();

        RED_GOAL_POSE = new Pose(xTarget, yTarget, Math.toRadians(45));
        goalUsed = shootOnMove ? futureGoal : RED_GOAL_POSE;
        distFromGoal = currentPose.distanceFrom(goalUsed);

        keepLauncherWarm();
        if (usePDF) usePD();
        else updateTurret();

        Log.i("Launcher: ", "Dist from goal: " + distFromGoal);
        Log.i("Launcher: ", "shooting on move: " + shootOnMove);
    }

    public void keepLauncherWarm(){
        BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distFromGoal);
        double flywheelSpeed = ballLaunchParameters.flywheelVelocity;
        robotHardware.setFlywheelVelocityInTPS(flywheelSpeed); // set the speed of the flywheel

        Log.i("Launcher: "," Target Velocity: " + flywheelSpeed);
    }

    public double calculateAmountToTurnToGoal(){
        double amountToTurn = Math.atan2(
                goalUsed.getY() - currentPose.getY(),
                goalUsed.getX() - currentPose.getX()
        );
        return Math.toDegrees(amountToTurn - currentPose.getHeading());
    }

    public double degreesToTicks(double deg){
        return (deg/360) * turretTicksPerRev;
    }

    public void updateTurret(){
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double deg = calculateAmountToTurnToGoal();
        if(Math.abs(deg) > turretLimit) return;
        int ticks = (int) degreesToTicks(deg);

        turret.setDirection(ticks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        Log.i("Turret: ", "Current Direction: " + turret.getDirection());

        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
        turret.setVelocity(velocity);
    }

    public void usePD(){
        double deg = calculateAmountToTurnToGoal();

        if(Math.abs(deg) > turretLimit) return;

        double targetTicks = degreesToTicks(deg);
        double currentTicks = turret.getCurrentPosition();
        double error = targetTicks - currentTicks;

        if(Math.abs(error) < 5){turret.setPower(0); return;}

        turret.setDirection(targetTicks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);

        double dt = pdTimer.seconds();
        pdTimer.reset();

        if(dt <= 0) return;

        double derivative = (error - lastError) / dt;

        double heading = currentPose.getHeading();
        double headingVelocity = (heading - lastHeading) / dt;

        double ff = headingVelocity * kF * turretTicksPerRev;

        lastHeading = heading;

        double output = (kP * error) + (kD * derivative) + ff;

        output = Range.clip(output, -maxTurretPower, maxTurretPower);

        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(Math.abs(output));

        lastError = error;

        Log.i("TurretPD", "Error: " + error);
        Log.i("TurretPD", "Output: " + output);
        Log.i("TurretPD", "FF: " + ff);
        Log.i("TurretPD ", "Turning direction: " + turret.getDirection().name());
    }

    public List<Double> getVisorPoses(){
        BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distFromGoal);
        Log.i("Launcher: ", "Visor poses: " + ballLaunchParameters.visorPositions);
        return ballLaunchParameters.visorPositions;
    }

    public Command moveKicker(){
        return new SequentialCommandGroup(
                new WaitCommand(150),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING)),
                new WaitCommand(150),
                getExactShotTime(),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(100)
        );
    }

    public Command shootAll(){
        List<Double> visorPoses = getVisorPoses();
        timer.reset();
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_1)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(0))),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_3)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(1))),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_2)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(2))),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
        );
    }

    public void shootOnMove(){
        currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double shootTimeMs;
        switch(spindex.getCurrentPose()){
            case LAUNCH_POSE_2: shootTimeMs = shot2Ms; break;
            case LAUNCH_POSE_3: shootTimeMs = shot3Ms; break;
            default: shootTimeMs = shot1Ms; break;
        }
        double t = shootTimeMs / 1000.0;

        futureGoal = new Pose(
                RED_GOAL_POSE.getX() - velocity.getXComponent() * t,
                RED_GOAL_POSE.getY() - velocity.getYComponent() * t
        );
    }

    public void setShootOnMove(boolean shootOnMove){this.shootOnMove = shootOnMove;}

    public Command getExactShotTime(){
        return new ParallelCommandGroup(
                new InstantCommand(() -> Log.i("Launcher: ", "time taken for shot: " + timer.milliseconds())),
                new InstantCommand(() -> robotHardware.setAlignmentLightColor(0.5))
        );
    }
    public Command shootColor(SpindexSort spindexSort){
        List<Double> visorPoses = getVisorPoses();
        return new SequentialCommandGroup(
                spindexSort.moveToColor(motif.get(0)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(0))),
                moveKicker(),
                spindexSort.moveToColor(motif.get(1)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(1))),
                moveKicker(),
                spindexSort.moveToColor(motif.get(2)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(2))),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
        );
    }
}