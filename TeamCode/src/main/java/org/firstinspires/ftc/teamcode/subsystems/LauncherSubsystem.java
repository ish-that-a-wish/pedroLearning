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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.common.BallLaunchParameters;
import org.firstinspires.ftc.teamcode.common.LaunchParametersLookup;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;

@Config
public class LauncherSubsystem extends SubsystemBase {
    public ElapsedTime timer = new ElapsedTime();
    public static double shootTimeMs = 200;
    private final double turretTicksPerRev = 751.8 * 5; // for torque increase
    public static double xTarget = 133;
    public static double yTarget = 137;

    public RobotHardware robotHardware;
    private Pose RED_GOAL_POSE = new Pose(xTarget, yTarget, Math.toRadians(45)); // pose of the red goal;
    // can adjust later
    public Pose midPoint = new Pose(133, 137, Math.toRadians(45));
    public Pose farPoint = new Pose(133, 137, Math.toRadians(45));
    public Pose shortPoint = new Pose(133, 137, Math.toRadians(45));
    private Follower follower;
    public Pose currentPose;
    public double distFromGoal;
    private DcMotorEx turret;
    public static int topWait = 150;
    public static int midWait = 150;
    public static int bottomWait = 100;
    public HardwareMap hardwareMap;
    public static double power = 0.2;
    public static int velocity = 2000;
    public SpindexSubsystem spindex;

    public enum Distance{
        SHORT,
        MID,
        FAR
    }
    public Distance currentDistance;
    public Pose futurePose; // defualt to it being noting
    private double amountToTurn;
    private boolean shootOnMove = false;
    private Pose poseUsed;
    public LauncherSubsystem(RobotHardware robotHardware, Follower follower, HardwareMap hardwareMap, SpindexSubsystem spindex){
        this.spindex = spindex;
        this.hardwareMap = hardwareMap;
        this.follower = follower;
        this.robotHardware = robotHardware;

        currentPose = follower.getPose();

        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPower(power);
    }

    public void update(){
        currentPose = follower.getPose();
        if(shootOnMove) shootOnMove();

        poseUsed = shootOnMove ? futurePose : currentPose;
        distFromGoal = poseUsed.distanceFrom(RED_GOAL_POSE);

        keepLauncherWarm();
        updateTurret();
        changeGoalByDist();

        Log.i("Launcher: ", "Dist from goal: " + distFromGoal);
        Log.i("Launcher: ", "shooting on move: " + shootOnMove);
        Log.i("Launcher: ", "Current Pose: " + (shootOnMove ? futurePose : currentPose));
    }

    public void keepLauncherWarm(){
        BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distFromGoal);
        double flywheelSpeed = ballLaunchParameters.flywheelVelocity;
        robotHardware.setFlywheelVelocityInTPS(flywheelSpeed); // set the speed of the flywheel

        Log.i("Launcher: "," Target Velocity: " + flywheelSpeed);
    }

    public double calculateAmountToTurnToGoal(){
        amountToTurn = Math.atan2(
                RED_GOAL_POSE.getY() - poseUsed.getY(),
                RED_GOAL_POSE.getX() - poseUsed.getX()
        );
        return Math.toDegrees(amountToTurn - poseUsed.getHeading());
    }
    public double degreesToTicks(double deg){
        return (deg/360) * turretTicksPerRev;
    }
    public void updateTurret(){
        double deg = calculateAmountToTurnToGoal();
        if(deg > 90) return;
        if (deg < -90) return;
        int ticks = (int) degreesToTicks(deg);

        turret.setDirection(ticks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        Log.i("Turret: ", "Current Direction: " + turret.getDirection());

        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(velocity);
        follower.getVelocity();
    }
    public List<Double> getVisorPoses(){
        BallLaunchParameters ballLaunchParameters = LaunchParametersLookup.getBallLaunchParameters(distFromGoal);
        Log.i("Launcher: ", "Visor poses: " + ballLaunchParameters.visorPositions);
        return ballLaunchParameters.visorPositions;
    }
    public Command moveKicker(){
        return new SequentialCommandGroup(
                new WaitCommand(topWait),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING)),
                new WaitCommand(midWait),
                new InstantCommand(() -> robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(bottomWait)
        );
    }
    public Command shootAll(){
        List<Double> visorPoses = getVisorPoses();
        timer.reset();
        return new SequentialCommandGroup(
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_1)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(0))),
                new InstantCommand(() -> getTimeToShoot(LAUNCH_POSE_1)),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_3)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(1))),
                new InstantCommand(() -> getTimeToShoot(LAUNCH_POSE_3)),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_2)),
                new InstantCommand(() -> robotHardware.setLaunchVisorPosition(visorPoses.get(2))),
                new InstantCommand(() -> getTimeToShoot(LAUNCH_POSE_2)),
                moveKicker(),
                new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
        );
    }

    public void changeGoalByDist(){
        if(distFromGoal < 40) currentDistance = Distance.SHORT;
        else if (distFromGoal > 72) currentDistance = Distance.FAR;
        else currentDistance = Distance.MID;

        // depending on the distance change the point
        switch(currentDistance){
            case FAR:
                RED_GOAL_POSE = farPoint;
                break;
            case MID:
                RED_GOAL_POSE = midPoint;
                break;
            case SHORT:
                RED_GOAL_POSE = shortPoint;
        }
    }

    // to shoot on the move, take into account the current velocity of our robot
    // & the amount of time it takes to shoot 3 artifacts
    public void shootOnMove(){
        currentPose = follower.getPose();
        Vector velocity = follower.getVelocity();
        double t = shootTimeMs / 1000.0; // convert ms -> seconds

        futurePose = new Pose(
                currentPose.getX() + velocity.getXComponent() * t,
                currentPose.getY() + velocity.getYComponent() * t,
                currentPose.getHeading() + velocity.getTheta() * t
        );
    }
    public void setShootOnMove(boolean shootOnMove){this.shootOnMove = shootOnMove;}
//    public boolean getShootOnMove(){return shootOnMove;}
    //get the spindex pose to see how long it takes for each diff pos
    public void getTimeToShoot(SpindexSubsystem.SpindexPoses spindexPose){
        // go in 132 order because thats the way we shoot our balls
        timer.reset(); // reset the timer every diff check
        switch (spindexPose){
            case LAUNCH_POSE_1:
                Log.i("Launcher: ", "Time to shoot first ball: " + timer.milliseconds());
            case LAUNCH_POSE_3:
                Log.i("Launcher: ", "Time to shoot second ball: " + timer.milliseconds());
            case LAUNCH_POSE_2:
                Log.i("Launcher: ", "Time to shoot third ball: " + timer.milliseconds());
        }
    }
}
