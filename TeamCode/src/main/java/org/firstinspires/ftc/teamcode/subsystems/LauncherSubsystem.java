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
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.openftc.apriltag.AprilTagDetection;

import java.util.List;

@Config
public class LauncherSubsystem extends SubsystemBase {
    //Basic required subsys
    public RobotHardware robotHardware;
    private Follower follower;
    public SpindexSubsystem spindex;


    // Goal following vars
    private Pose RED_GOAL_POSE = new Pose(133, 137, Math.toRadians(45)); // pose of the red goal;
    public Pose currentPose;
    public double distFromGoal;

    //turret specific vars
    private final double turretTicksPerRev = 751.8 * 5; // for torque increase
    private DcMotorEx turret;
    //SOTM
    private boolean shootOnMove = false;
    private Pose goalUsed;
    public static double shootTimeMs = 200;
    public Pose futureGoal; // defualt to it being noting

    // TURRET KP AND KD
    public static double kp = 0.001;
    public static double kd = 0.00;
    private double lastError = 0;
    public static double tolerance = 0.4; // set the tolerance to 0.4 deg
    private double power = 0;
    private final ElapsedTime timer = new ElapsedTime();
    public static boolean useTurretPD = false;
    public LauncherSubsystem(RobotHardware robotHardware, Follower follower, HardwareMap hardwareMap, SpindexSubsystem spindex){
        this.spindex = spindex;
        this.follower = follower;
        this.robotHardware = robotHardware;

        currentPose = follower.getPose();

        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPower(0.2);
    }

    public void update(){
        currentPose = follower.getPose();
        if(shootOnMove) shootOnMove();

        goalUsed = shootOnMove ? futureGoal : RED_GOAL_POSE;
        distFromGoal = currentPose.distanceFrom(goalUsed);

        keepLauncherWarm();
        if (useTurretPD) updateUsingPD();
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
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        double deg = calculateAmountToTurnToGoal();
        if(deg > 90) return;
        if (deg < -90) return;
        int ticks = (int) degreesToTicks(deg);

        turret.setDirection(ticks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        Log.i("Turret: ", "Current Direction: " + turret.getDirection());

        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(2000);
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
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(100)
        );
    }
    public Command shootAll(){
        List<Double> visorPoses = getVisorPoses();
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

    // to shoot on the move, take into account the current velocity of our robot
    // & the amount of time it takes to shoot 3 artifacts
    public void shootOnMove(){
        currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double t = shootTimeMs / 1000.0; // convert ms -> seconds

        futureGoal = new Pose(
                RED_GOAL_POSE.getX() - velocity.getXComponent() * t,
                RED_GOAL_POSE.getY() - velocity.getYComponent() * t
        );
    }
    public void setShootOnMove(boolean shootOnMove){this.shootOnMove = shootOnMove;}
//    public boolean getShootOnMove(){return shootOnMove;}
    //get the spindex pose to see how long it takes for each diff pos
    public void updateUsingPD(){
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double deltaTime = timer.seconds();
        if(deltaTime == 0) return;
        timer.reset();

        double error = calculateAmountToTurnToGoal();
        if(Math.abs(error) > 90 || Math.abs(error) < tolerance) return; // turret limits

        double pTerm = error * kp;
        double dTerm = 0;
        if(deltaTime > 0) dTerm = ((error - lastError ) / deltaTime) * kd;

        power = Range.clip(pTerm + dTerm, -1, 1); // set the power to max out at 1, can add max power later

        turret.setPower(power);
        lastError = error;
    }
}
