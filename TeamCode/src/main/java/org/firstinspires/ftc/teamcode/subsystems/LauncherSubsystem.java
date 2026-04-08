package org.firstinspires.ftc.teamcode.subsystems;
import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.*;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.*;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.MathFunctions;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.Actions.Commands.MoveSpindexCommand;
import org.firstinspires.ftc.teamcode.TeleOp.TeleopTestingForTurret;
import org.firstinspires.ftc.teamcode.common.*;

import java.util.ArrayList;
import java.util.Calendar;
import java.util.Date;
import java.util.List;

@Config
public class LauncherSubsystem extends SubsystemBase {
    private final double HOOD_MIN = 0.01;
    private final double HOOD_MAX = 0.68;
    private final double HOOD_MIN_ANGLE = 15;
    private final double HOOD_MAX_ANGLE = 60;

    // ---------------- CORE ----------------
    private final RobotHardware robot;
    private final Follower follower;
    private final SpindexSubsystem spindex;
    private final DcMotorEx turret;

    // ---------------- TARGET ----------------
    public static double xTarget = 130, yTarget = 133;
    private Pose currentGoalPose = new Pose(xTarget, yTarget, Math.toRadians(45));
    private Pose futureGoalPose;

    private Pose currentPose;
    private double distFromGoal;

    // ---------------- TURRET ----------------
    private final double turretTicksPerRev = 751.8 * 5;
    public static double turretLimit = 120;
    public static double velocity = 4000;

    // ---------------- SHOOTING ----------------
    public static double shot1Ms = 500, shot2Ms = 1500, shot3Ms = 1500;
    private boolean shootOnMove = true;
    public ElapsedTime launchTimer = new ElapsedTime();
    public static long kickerTime = 150;

    private final double ballRadius = 2.5; // in inches
    private final double robotHeight = 14; // in inches
    public static double goalHeight = 38.75; // in inches
    private final double GRAVITY = 9.81;
    public static double HORIZONTAL_FLYWHEEL_SPINDEX_1_MULTIPLIER = 0.11;
    public static double HORIZONTAL_FLYWHEEL_SPINDEX_2_MULTIPLIER = 0.11;
    public static double HORIZONTAL_FLYWHEEL_SPINDEX_3_MULTIPLIER = 0.11;
    public static double VERTICAL_FLYWHEEL_SPINDEX_1_MULTIPLIER = 0.21;
    public static double VERTICAL_FLYWHEEL_SPINDEX_2_MULTIPLIER = 0.25;
    public static double VERTICAL_FLYWHEEL_SPINDEX_3_MULTIPLIER = 0.12;


    public static double buffer = 10;

    public LauncherSubsystem(RobotHardware robot, Follower follower,
                             HardwareMap hardwareMap, SpindexSubsystem spindex) {

        this.robot = robot;
        this.follower = follower;
        this.spindex = spindex;

        currentPose = follower.getPose();

        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ---------------- UPDATE ----------------

    public void update() {
        currentPose = follower.getPose();

//        updateGoal();
//        updateDistance();
//        updateFlywheel();
//        updateTurret();
        updateAdvanced();
        Log.i("===LAUNCHER SUBSYS===", "Current pose: " + follower.getPose());
        Log.i("===LAUNCHER SUBSYS=== ", "Dist: " + distFromGoal);
    }

    private void updateGoal() {
        currentGoalPose = new Pose(xTarget, yTarget, Math.toRadians(45));

        if (!shootOnMove) {
            futureGoalPose = currentGoalPose;
            return;
        }

        Vector vel = follower.getVelocity();

        double t = getShotTime() / 1000.0;
//        if (launchStarted) t = Math.max(0, (getShotTime() - launchTimer.milliseconds()) / 1000);

        //code may have a error, as t->0 we get closer and closer to real goal pose with no real vel compensation
        //may have to do this instead
        //double t = getshotTime()/1000
        //if(!launchStarted){
//        futureGoalPose = new Pose(
//                currentGoalPose.getX() - vel.getXComponent() * t,
//                currentGoalPose.getY() - vel.getYComponent() * t
//        );
        //without the launchTimer at all
//      }
        Log.i("===LAUNCHER SUBSYS===", "updateGoal. Current Pose X: " + currentPose.getX() + " Y: " + currentPose.getY());
        Log.i("===LAUNCHER SUBSYS===", "updateGoal. Current Velocity. X: " + vel.getXComponent() + " Y: " + vel.getYComponent());
        futureGoalPose = new Pose(
                currentGoalPose.getX() - vel.getXComponent() * t,
                currentGoalPose.getY() - vel.getYComponent() * t
        );
        Log.i("===LAUNCHER SUBSYS===", "updateGoal. Launch timer: " + launchTimer.milliseconds());
        Log.i("===LAUNCHER SUBSYS===", "updateGoal. Future Pose X: " + futureGoalPose.getX() + " Y: " + futureGoalPose.getY());

    }

    private void updateDistance() {
        Pose goal = (futureGoalPose != null) ? futureGoalPose : currentGoalPose;
        distFromGoal = currentPose.distanceFrom(goal);
    }

    // ---------------- FLYWHEEL ----------------

    private void updateFlywheel() {
        double speed = LaunchParametersLookup
                .getBallLaunchParameters(distFromGoal)
                .flywheelVelocity;

        robot.setFlywheelVelocityInTPS(speed);
    }

    // ---------------- TURRET ----------------

    private double getAngleToGoal() {
        Pose goal = (futureGoalPose != null) ? futureGoalPose : currentGoalPose;

        double angle = Math.atan2(
                goal.getY() - currentPose.getY(),
                goal.getX() - currentPose.getX()
        );

        return Math.toDegrees(angle - currentPose.getHeading());
    }

    private double degreesToTicks(double deg) {
        return (deg / 360.0) * turretTicksPerRev;
    }

    public void updateTurret() {
        double deg = getAngleToGoal();

        if (Math.abs(deg) > turretLimit) {
            turret.setPower(0);
            return;
        }

        int ticks = (int) degreesToTicks(deg);

        turret.setTargetPosition(ticks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(velocity);

    }

    // ---------------- SHOOTING ----------------

    private double getShotTime() {
        if (spindex.getCurrentPose() != null) {
            switch (spindex.getCurrentPose()) {
                case LAUNCH_POSE_2:
                    return shot2Ms;
                case LAUNCH_POSE_3:
                    return shot3Ms;
                default:
                    return shot1Ms;
            }
        } else {
            return shot1Ms; // always js defualt to shot 1
        }
    }

    public Command moveKicker() {
        return new SequentialCommandGroup(
                new InstantCommand(() -> robot.setLaunchKickPosition(LAUNCH_KICK_KICKING)),
                new WaitCommand(kickerTime),
                new InstantCommand(() -> Log.i("===LAUNCHER SUBSYS", "FINISHED KICKING BALL UP")),
                new InstantCommand(() -> robot.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(kickerTime)
        );
    }


    public Command shootAll() {
        List<Double> visor = getVisorPoses();

        return new SequentialCommandGroup(
                new InstantCommand(() -> launchTimer.reset()),
                new InstantCommand(() -> Log.i("===LAUNCHER SUBSYS===", "STARTING LAUNCHING BALL 1")),
                new InstantCommand(() -> robot.setLaunchVisorPosition(visor.get(0))),
                new MoveSpindexCommand(spindex, robot, LAUNCH_POSE_1),
                moveKicker(),
                new InstantCommand(() -> launchTimer.reset()),
                new InstantCommand(() -> Log.i("===LAUNCHER SUBSYS===", "STARTING LAUNCHING BALL 2")),
                new InstantCommand(() -> robot.setLaunchVisorPosition(visor.get(1))),
                new MoveSpindexCommand(spindex, robot, LAUNCH_POSE_3),
                moveKicker(),
                new InstantCommand(() -> launchTimer.reset()),
                new InstantCommand(() -> Log.i("===LAUNCHER SUBSYS===", "STARTING LAUNCHING BALL 3")),
                new InstantCommand(() -> robot.setLaunchVisorPosition(visor.get(2))),
                new MoveSpindexCommand(spindex, robot, LAUNCH_POSE_2),
                moveKicker(),
                new InstantCommand(() -> Log.i("===LAUNCHER SUBSYS===", "FINISHED LAUNCHING")),
                new MoveSpindexCommand(spindex, robot, INTAKE_POSE_1)
        );
    }


    // ---------------- VISOR ----------------

    public List<Double> getVisorPoses() {
        return LaunchParametersLookup
                .getBallLaunchParameters(distFromGoal)
                .visorPositions;
    }

    public List<Double> predictAdvanced(Pose robotPose, Vector robotSpeed, Vector robotAcceleration) {
        List<Double> result = new ArrayList<>();
        final double targetX = currentGoalPose.getX() - Math.signum(currentGoalPose.getX()) * ballRadius;
        final double targetY = currentGoalPose.getY() - ballRadius;
        final double targetZ = goalHeight - robotHeight - ballRadius;
        final double robotX = robotPose.getX();
        final double robotY = robotPose.getY();


        final double inchesPerMeters = 39.3701;
        final double gravityInches = GRAVITY * inchesPerMeters;

        final double flightTime = Math.sqrt(2 * (targetZ) / gravityInches);

        // 📍 Project robot position forward using x + vt + ½at²
        final double futureRobotX = robotX + robotSpeed.getXComponent() * flightTime
                + 0.5 * robotAcceleration.getXComponent() * Math.pow(flightTime, 2);
        final double futureRobotY = robotY + robotSpeed.getYComponent() * flightTime
                + 0.5 * robotAcceleration.getYComponent() * Math.pow(flightTime, 2);

        // 🎯 Recalculate target direction from future robot position
        final double correctedDistToTarget = Math.sqrt(Math.pow(targetX - futureRobotX, 2) + Math.pow(targetY - futureRobotY, 2));
        final double correctedYawAngle = Math.atan2((targetY - futureRobotY), (targetX - futureRobotX));
        final double correctedTargetVelX = Math.cos(correctedYawAngle) * (correctedDistToTarget / flightTime);
        final double correctedTargetVelY = Math.sin(correctedYawAngle) * (correctedDistToTarget / flightTime);

        // 🚀 Project robot velocity forward using v + at
        final double effectiveRobotVelX = robotSpeed.getXComponent() + robotAcceleration.getXComponent() * flightTime;
        final double effectiveRobotVelY = robotSpeed.getYComponent() + robotAcceleration.getYComponent() * flightTime;

        final double relativeVelX = correctedTargetVelX - effectiveRobotVelX;
        final double relativeVelY = correctedTargetVelY - effectiveRobotVelY;
        final double relativeHorizontalVel = Math.sqrt(Math.pow(relativeVelX, 2) + Math.pow(relativeVelY, 2));
        final double velocityCompensatedDistance = relativeHorizontalVel * flightTime;

        Log.i("===LAUNCHER===", "With advanced: " + "Distance with vel comp: " + velocityCompensatedDistance);

        final double turretAngle = Math.toDegrees(Math.atan2(relativeVelY, relativeVelX) - robotPose.getHeading());
        BallLaunchParameters results = LaunchParametersLookup.getBallLaunchParameters(velocityCompensatedDistance);
        distFromGoal = velocityCompensatedDistance;


        // plz speed work
        if (robotSpeed.getYComponent() < 0 && robotSpeed.getYComponent() > buffer) {
            Log.i("===ROBOT===", "RUNNING HORIZONTAL FLYWHEEL MULTIPLIER");
            double multiplier = VERTICAL_FLYWHEEL_SPINDEX_1_MULTIPLIER;
            switch(spindex.getCurrentPose()){
                case LAUNCH_POSE_1:
                    multiplier = VERTICAL_FLYWHEEL_SPINDEX_1_MULTIPLIER;
                    break;
                case LAUNCH_POSE_2:
                    multiplier = VERTICAL_FLYWHEEL_SPINDEX_2_MULTIPLIER;
                    break;
                case LAUNCH_POSE_3:
                    multiplier = VERTICAL_FLYWHEEL_SPINDEX_3_MULTIPLIER;
                    break;
            }
            results.flywheelVelocity *= multiplier * Math.abs(robotSpeed.getYComponent());
        } // if we slow put the multiplier really small, faster more
        if (robotSpeed.getXComponent() < 0 && Math.abs(robotSpeed.getXComponent()) > buffer) {
            Log.i("===ROBOT===", "RUNNING HORIZONTAL FLYWHEEL MULTIPLIER");
            double multiplier = HORIZONTAL_FLYWHEEL_SPINDEX_1_MULTIPLIER;
            switch(spindex.getCurrentPose()){
                case LAUNCH_POSE_1:
                    multiplier = HORIZONTAL_FLYWHEEL_SPINDEX_1_MULTIPLIER;
                    break;
                case LAUNCH_POSE_2:
                    multiplier = HORIZONTAL_FLYWHEEL_SPINDEX_2_MULTIPLIER;
                    break;
                case LAUNCH_POSE_3:
                    multiplier = HORIZONTAL_FLYWHEEL_SPINDEX_3_MULTIPLIER;
                    break;
            }
            results.flywheelVelocity *= multiplier * Math.abs(robotSpeed.getYComponent());
            results.flywheelVelocity *= multiplier * Math.abs(robotSpeed.getXComponent());
        }


        Log.i("===ROBOT=== ", "CURRENT VERTICAL SPEED: " + robotSpeed.getYComponent());
        Log.i("===ROBOT===", "CURRENT HORIZONTAL SPEED: " + robotSpeed.getXComponent());


        result.add(results.flywheelVelocity);
        result.add(turretAngle);
        result.add(results.visorPositions.get(0));
        result.add(results.visorPositions.get(1));
        result.add(results.visorPositions.get(2));
        return result;
    }

    public void updateAdvanced() {
        List<Double> launchParameters = predictAdvanced(
                follower.getPose(),
                follower.getVelocity(),
                follower.getAcceleration()  // 🆕 passed in here
        );
        Log.i("===LAUNCHER===", "With advanced: " + "Launcher speed: " + launchParameters.get(0));
        Log.i("===LAUNCHER===", "With advanced: " + "Turret angle: " + launchParameters.get(1));
        Log.i("===LAUNCHER===", "With advanced: " + "Hood angle: " + launchParameters.get(2));
        Log.i("===LAUNCHER===", "With advanced: " + "Real flywheel speed: " + robot.getFlywheelVelocityInTPS());
        updateTurret(launchParameters.get(1));
        robot.setFlywheelVelocityInTPS(launchParameters.get(0));
        robot.setLaunchVisorPosition(launchParameters.get(2));
        currentGoalPose = new Pose(xTarget, yTarget);
    }
    public void updateHood(double deg) {
        double hoodPosToDeg = (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE) / (HOOD_MAX - HOOD_MIN);
        double degPos = deg * hoodPosToDeg;
        double finalPos = MathFunctions.clamp(degPos, HOOD_MIN, HOOD_MAX);
        Log.i("===LAUNCHER===", "With advanced: " + "Hood pos: " + finalPos);//        robot.setLaunchVisorPosition(finalPos);    }`
    }
    public void updateTurret(double deg){
        if (Math.abs(deg) > turretLimit) {
            turret.setPower(0);
            return;
        }

        int ticks = (int) degreesToTicks(deg);

        turret.setTargetPosition(ticks);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(velocity);
    }
}