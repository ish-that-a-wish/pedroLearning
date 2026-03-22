package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.*;
import static org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem.SpindexPoses.*;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.common.*;

import java.util.ArrayList;
import java.util.List;

@Config
public class LauncherSubsystem extends SubsystemBase {

    // ---------------- CORE ----------------
    private final RobotHardware robot;
    private final Follower follower;
    private final SpindexSubsystem spindex;
    private final DcMotorEx turret;

    // ---------------- TARGET ----------------
    public static double xTarget = 131, yTarget = 138;
    private Pose goalPose = new Pose(xTarget, yTarget, Math.toRadians(45));
    private Pose futureGoal = null;

    private Pose currentPose;
    private double distFromGoal;

    // ---------------- TURRET ----------------
    private final double turretTicksPerRev = 751.8 * 5;
    public static double turretLimit = 120;
    public static double power = 0.8, velocity = 4000;

    // ---------------- SHOOTING ----------------
    public static double shot1Ms = 350, shot2Ms = 570, shot3Ms = 470;
    private boolean shootOnMove = true;

    private final ElapsedTime timer = new ElapsedTime();

    public List<GameColors> motif = new ArrayList<>();

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

        // default motif
        motif.add(GameColors.PURPLE);
        motif.add(GameColors.GREEN);
        motif.add(GameColors.PURPLE);
    }

    // ---------------- UPDATE ----------------

    public void update(boolean sort) {
        currentPose = follower.getPose();

        updateGoal(sort);
        updateDistance();
        updateFlywheel();
        updateTurret();

        Log.i("Launcher", "Dist: " + distFromGoal);
    }

    private void updateGoal(boolean sort) {
        goalPose = new Pose(xTarget, yTarget, Math.toRadians(45));

        if (!shootOnMove || sort) {
            futureGoal = goalPose;
            return;
        }

        Vector vel = follower.getVelocity();
        double t = getShotTime() / 1000.0;

        futureGoal = new Pose(
                goalPose.getX() - vel.getXComponent() * t,
                goalPose.getY() - vel.getYComponent() * t
        );
    }

    private void updateDistance() {
        Pose goal = (futureGoal != null) ? futureGoal : goalPose;
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
        Pose goal = (futureGoal != null) ? futureGoal : goalPose;

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
            turret.setPower(0); // 🔥 prevent jitter at limits
            return;
        }

        int ticks = (int) degreesToTicks(deg);

        turret.setDirection(ticks < 0
                ? DcMotorSimple.Direction.FORWARD
                : DcMotorSimple.Direction.REVERSE);

        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(power);
        turret.setVelocity(velocity);
    }

    // ---------------- SHOOTING ----------------

    private double getShotTime() {
        switch (spindex.getCurrentPose()) {
            case LAUNCH_POSE_2: return shot2Ms;
            case LAUNCH_POSE_3: return shot3Ms;
            default: return shot1Ms;
        }
    }

    public Command moveKicker() {
        return new SequentialCommandGroup(
                new WaitCommand(120),
                new InstantCommand(() -> robot.setLaunchKickPosition(LAUNCH_KICK_KICKING)),
                new WaitCommand(120),
                new InstantCommand(() -> robot.setLaunchKickPosition(LAUNCH_KICK_RESTING)),
                new WaitCommand(80)
        );
    }

    private Command shootSequence(List<Double> visorPoses, List<Command> moves) {
        List<Command> commands = new ArrayList<>();

        for (int i = 0; i < 3; i++) {
            int idx = i;

            commands.add(moves.get(i));
            commands.add(new InstantCommand(() ->
                    robot.setLaunchVisorPosition(visorPoses.get(idx))
            ));
            commands.add(moveKicker());
        }

        return new SequentialCommandGroup(commands.toArray(new Command[0]));
    }

    public Command shootAll() {
        List<Double> visor = getVisorPoses();

        List<Command> moves = List.of(
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_1)),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_3)),
                new InstantCommand(() -> spindex.moveToPose(LAUNCH_POSE_2))
        );

        return new SequentialCommandGroup(
                shootSequence(visor, moves),
                new InstantCommand(() -> spindex.moveToPose(INTAKE_POSE_1))
        );
    }

    public void shootColor(SpindexSort sort) {
        List<Double> visor = getVisorPoses();

        List<Command> moves = List.of(
                sort.moveToColor(motif.get(0)),
                sort.moveToColor(motif.get(1)),
                sort.moveToColor(motif.get(2))
        );

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        shootSequence(visor, moves),

                        new InstantCommand(sort::finishLaunch),

                        new InstantCommand(() -> sort.moveToPose(0, true))
                )
        );
    }

    // ---------------- VISOR ----------------

    public List<Double> getVisorPoses() {
        return LaunchParametersLookup
                .getBallLaunchParameters(distFromGoal)
                .visorPositions;
    }

    // ---------------- SETTINGS ----------------

    public void setShootOnMove(boolean enabled) {
        this.shootOnMove = enabled;
    }
}