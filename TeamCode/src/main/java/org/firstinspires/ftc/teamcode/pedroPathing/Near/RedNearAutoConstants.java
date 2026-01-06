package org.firstinspires.ftc.teamcode.pedroPathing.Near;

import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;

public class RedNearAutoConstants {
    public static int multiplier = 1;
    public static int midLine = 72;
    /* ------------------------------------------------------------
     * POSES
     * ------------------------------------------------------------ */

    public static Pose INIT_POSE = new Pose(midLine + (multiplier * 52), 125, Math.toRadians(90 - (multiplier * 54)));
    public static Pose FIRST_SPIKE = new Pose(midLine + (45 * multiplier), 84, Math.toRadians(90 - (multiplier * 90)));
    public static Pose SECOND_SPIKE = new Pose(midLine + (45 * multiplier), 60, Math.toRadians(90 - (multiplier * 90)));
    public static Pose SHOOTING_POSE = new Pose(midLine + (22 * multiplier), 84, Math.toRadians(90 - (multiplier * 90)));

    public static Pose GATE_PICKUP = new Pose(midLine + (58 * multiplier), 63, Math.toRadians(90 - (multiplier * 45)));
    public static Pose GATE_PICKUP_NO_INTAKE = new Pose(midLine + (62 * multiplier), 67, Math.toRadians(90 - (multiplier * 90)));

    public static Pose PRE_HUMAN_PLAYER = new Pose(midLine + (57 * multiplier), 30, Math.toRadians(270));
    public static Pose HUMAN_PLAYER_PICKUP = new Pose(midLine + (57 * multiplier), 20, Math.toRadians(270));

    public static Pose SECRET_TUNNEL_AFTER_GATE = new Pose(midLine + (62 * multiplier), 45, Math.toRadians(90 - (multiplier * 90)));

    public static Pose MOVE_OFF_LINE = new Pose(midLine + (17 * multiplier), 115);

    /* Control points for curves */
    private static Pose CONTROL_POINT_FIRST_SPIKE = new Pose(midLine + (25 * multiplier), 82);
    private static Pose CONTROL_POINT_SECOND_SPIKE = new Pose(midLine + (17*multiplier), 55);
    private static Pose CONTROL_POINT_SECRET_TUNNEL = new Pose(60, 60);
    public static Pose SECRET_TUNNEL_CONTROL_AFTER_GATE = new Pose(midLine + (42*multiplier), 50);

    /* ------------------------------------------------------------
     * PATH OBJECTS
     * ------------------------------------------------------------ */

    // Individual reusable paths

    public static Path shootPreload =
            new Path(new BezierLine(INIT_POSE, SHOOTING_POSE));

    public static Path moveToSpike1Pickup =
            new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_FIRST_SPIKE, FIRST_SPIKE));

    public static Path moveToShootSpike1 =
            new Path(new BezierLine(FIRST_SPIKE, SHOOTING_POSE));

    public static Path moveToSpike2Pickup =
            new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECOND_SPIKE, SECOND_SPIKE));

    public static Path moveToShootSpike2 =
            new Path(new BezierLine(SECOND_SPIKE, SHOOTING_POSE));

    public static Path moveToGate =
            new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP));

    public static Path moveToShootGate =
            new Path(new BezierLine(GATE_PICKUP, SHOOTING_POSE));

    public static Path preHumanPlayer =
            new Path(new BezierLine(SHOOTING_POSE, PRE_HUMAN_PLAYER));

    public static Path moveToHumanPlayer =
            new Path(new BezierLine(PRE_HUMAN_PLAYER, HUMAN_PLAYER_PICKUP));

    public static Path moveToShootHumanPlayer =
            new Path(new BezierLine(HUMAN_PLAYER_PICKUP, SHOOTING_POSE));

    public static Path moveToGateNoIntake =
            new Path(new BezierLine(SHOOTING_POSE, GATE_PICKUP_NO_INTAKE));

    public static Path moveToShootGateNoIntake =
            new Path(new BezierLine(GATE_PICKUP_NO_INTAKE, SHOOTING_POSE));

    public static Path moveOffLaunchLine =
            new Path(new BezierLine(SHOOTING_POSE, MOVE_OFF_LINE));

    public static Path SecretTunnelAfterGate =
            new Path(new BezierCurve(
                    GATE_PICKUP_NO_INTAKE,
                    SECRET_TUNNEL_CONTROL_AFTER_GATE,
                    SECRET_TUNNEL_AFTER_GATE
            ));

    public static Path SecretTunnel =
            new Path(new BezierCurve(SHOOTING_POSE, CONTROL_POINT_SECRET_TUNNEL, SECRET_TUNNEL_AFTER_GATE));

    public static Path postSecretTunnel =
            new Path(new BezierLine(SECRET_TUNNEL_AFTER_GATE, SHOOTING_POSE));

}