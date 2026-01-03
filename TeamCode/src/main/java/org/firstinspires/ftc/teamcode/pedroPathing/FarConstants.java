package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
@Config
public class FarConstants {
    public static Pose INIT_POSE = new Pose(56, 8, Math.toRadians(180));
    public static Pose FIRST_SPIKE = new Pose(27, 35, Math.toRadians(180));
    public static Pose SECOND_SPIKE = new Pose(27, 60, Math.toRadians(180));

    public static Pose SHOOTING_POSE = new Pose(56, 18, Math.toRadians(180));
    public static Pose GATE_PICKUP = new Pose(14, 63, Math.toRadians(45));
    public static Pose GATE_PICKUP_NO_INTAKE = new Pose(10, 63, Math.toRadians(180));

    public static Pose PRE_HUMAN_PLAYER = new Pose(15, 30, Math.toRadians(270));
    public static Pose HUMAN_PLAYER_PICKUP = new Pose(15, 20, Math.toRadians(270));

    public static Pose SECRET_TUNNEL_AFTER_GATE = new Pose(10, 45, Math.toRadians(270));
    public static Pose SECRET_TUNNEL_INTAKE = new Pose(3, 32, Math.toRadians(270));
    public static Pose SECRET_TUNNEL_AFTER_SHOOTING = new Pose(14, 40, Math.toRadians(90));
    public static Pose CONTROL_SECRET_TUNNEL_AFTER_SHOOTING = new Pose(11, 2);

    private static Pose CONTROL_POINT_FIRST_SPIKE = new Pose(56, 40);
    public static Pose CONTROL_POINT_SECOND_SPIKE = new Pose(55, 63);
    public static Pose SECRET_TUNNEL_CONTROL_AFTER_GATE = new Pose(30, 50);
    public static Pose MOVE_OFF_LINE = new Pose(50, 24);
    public static Path moveToSpike1Pickup = new Path(new BezierCurve(INIT_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE));
    public static Path moveToShootSpike1 = new Path(new BezierLine(FIRST_SPIKE, SHOOTING_POSE));
    public static Path moveToSpike2Pickup = new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE));
    public static Path moveToShootSpike2 = new Path(new BezierLine(SECOND_SPIKE, SHOOTING_POSE));
    public static Path moveToGate = new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP));
    public static Path moveToShootGate = new Path(new BezierLine(GATE_PICKUP, SHOOTING_POSE));
    public static Path preHumanPlayer = new Path(new BezierLine(SHOOTING_POSE, PRE_HUMAN_PLAYER));
    public static Path moveToHumanPlayer = new Path(new BezierLine(PRE_HUMAN_PLAYER, HUMAN_PLAYER_PICKUP));
    public static Path moveToShootHumanPlayer = new Path(new BezierLine(HUMAN_PLAYER_PICKUP, SHOOTING_POSE));
    public static Path moveToGateNoIntake = new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP_NO_INTAKE));
    public static Path moveToShootGateNoIntake = new Path(new BezierLine(GATE_PICKUP_NO_INTAKE, SHOOTING_POSE));
    public static Path moveOffLaunchLine = new Path(new BezierLine(SHOOTING_POSE, MOVE_OFF_LINE));
    public static Path preSecretTunnelAfterGate = new Path(new BezierCurve(GATE_PICKUP_NO_INTAKE, SECRET_TUNNEL_CONTROL_AFTER_GATE, SECRET_TUNNEL_AFTER_GATE));
    public static Path postSecretTunnel = new Path(new BezierLine(SECRET_TUNNEL_AFTER_GATE, SHOOTING_POSE));
    public static Path moveToSecretTunnelAfterShooting = new Path(new BezierCurve(SHOOTING_POSE, CONTROL_SECRET_TUNNEL_AFTER_SHOOTING, SECRET_TUNNEL_AFTER_SHOOTING));
    public static Path moveToShootAfterSecretTunnelShooting = new Path(new BezierLine(SECRET_TUNNEL_AFTER_SHOOTING, SHOOTING_POSE));
}
