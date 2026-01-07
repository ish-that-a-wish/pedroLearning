package org.firstinspires.ftc.teamcode.pedroPathing;

import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.FarShotHeading;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.FarShotPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.GateHeading;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.GatePose;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.HumanPlayerIntakeHeading;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.HumanPlayerIntakePower;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.HumanPlayerPose;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.MoveToFarShootingPower;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.MoveToGatePower;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.MoveToNearShootingPower;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.NearShotHeading;
import static org.firstinspires.ftc.teamcode.pedroPathing.TeleOpConstants.NearShotPose;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Far.BlueFarConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Near.BlueNearAutoConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

@TeleOp(name = "TeleOpPedro")
public class TeleOpHold extends LinearOpMode {

    /* -------------------- State Flags -------------------- */
    public boolean isFollowingNear = false;
    public boolean isFollowingFar  = false;
    private boolean followingPath  = false;

    /* -------------------- Drive Hardware -------------------- */
    private DcMotorEx leftFront;
    private DcMotorEx rightFront;
    private DcMotorEx leftRear;
    private DcMotorEx rightRear;

    /* -------------------- Drive Inputs -------------------- */
    private double speed;
    private double strafe;
    private double turn;

    /* -------------------- Pedro Pathing -------------------- */
    private Follower follower;

    /* -------------------- Pose Offset (D-Pad) -------------------- */
    private double xAdd = 0;
    private double yAdd = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(new Pose(24, 72, Math.toRadians(90)));

        initialize();

        waitForStart();

        while (opModeIsActive()) {

            follower.update();

            /* -------------------- BUTTON PATH TRIGGERS -------------------- */

            if (gamepad1.bWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO NEAR SHOOTING POSE X:", String.valueOf(NearShotPose.getX() + xAdd));
                Log.i("MOVING TO NEAR SHOOTING POSE Y:", String.valueOf(NearShotPose.getY() + yAdd));

                gamepadStuff(MoveToNearShootingPower, NearShotPose, NearShotHeading); // pose is only for testing

                isFollowingNear = true;
                followingPath   = true;
            }

            if (gamepad1.yWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO HUMAN POSE X:", String.valueOf(HumanPlayerPose.getX() + xAdd));
                Log.i("MOVING TO HUMAN POSE Y:", String.valueOf(HumanPlayerPose.getY() + yAdd));

                gamepadStuff(HumanPlayerIntakePower, HumanPlayerPose, HumanPlayerIntakeHeading);

                followingPath = true;
            }

            if (gamepad1.aWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO FAR SHOOTING POSE X:", String.valueOf(FarShotPose.getX() + xAdd));
                Log.i("MOVING TO FAR SHOOTING POSE Y:", String.valueOf(FarShotPose.getY() + yAdd));

                gamepadStuff(MoveToFarShootingPower, FarShotPose, FarShotHeading);

                isFollowingFar = true;
                followingPath  = true;
            }

            if (gamepad1.xWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO GATE POSE X:", String.valueOf(GatePose.getX()));
                Log.i("MOVING TO GATE POSE Y:", String.valueOf(GatePose.getY()));

                gamepadStuff(MoveToGatePower, GatePose, GateHeading);

                followingPath = true;
            }

            /* -------------------- D-PAD OFFSET UPDATES -------------------- */

            boolean poseChanged = moveThroughDpad();

            if (poseChanged && followingPath && isFollowingNear) {
                gamepadStuff(
                        MoveToNearShootingPower,
                        new Pose(
                                NearShotPose.getX() + xAdd,
                                NearShotPose.getY() + yAdd
                        ),
                        NearShotHeading
                );
            }

            if (poseChanged && followingPath && isFollowingFar) {
                gamepadStuff(
                        MoveToFarShootingPower,
                        new Pose(
                                FarShotPose.getX() + xAdd,
                                FarShotPose.getY() + yAdd
                        ),
                        FarShotHeading
                );
            }

            /* -------------------- CANCEL PATH -------------------- */

            if (gamepad1.rightBumperWasReleased()) {
                Log.i("FOLLOWING:", "PAUSING PATH");

                isFollowingNear = false;
                isFollowingFar  = false;
                followingPath   = false;

                follower.pausePathFollowing();
                follower.setMaxPower(0);
            }

            /* -------------------- MANUAL DRIVE -------------------- */

            if (!followingPath) {
                Log.i("FOLLOWING:", "CONTROLLER MOVEMENT");

                isFollowingNear = false;
                isFollowingFar  = false;

                speed  = -gamepad1.left_stick_y;
                strafe =  gamepad1.left_stick_x;
                turn   =  gamepad1.right_stick_x;

                moveWheels();
            }
        }
    }

    /* ==================== DRIVE METHODS ==================== */

    public void moveWheels() {
        double lf = speed + strafe + turn;
        double rf = speed - strafe - turn;
        double lr = speed - strafe + turn;
        double rr = speed + strafe - turn;

        double max = Math.max(
                1.0,
                Math.max(
                        Math.abs(lf),
                        Math.max(Math.abs(rf), Math.max(Math.abs(lr), Math.abs(rr)))
                )
        );

        leftFront.setPower(lf / max);
        rightFront.setPower(rf / max);
        leftRear.setPower(lr / max);
        rightRear.setPower(rr / max);
    }

    /* ==================== INIT ==================== */

    public void initialize() {
        leftFront  = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftRear   = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightRear  = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /* ==================== D-PAD OFFSET ==================== */

    public boolean moveThroughDpad() {
        boolean changed = false;

        if (gamepad1.dpadUpWasReleased()) {
            yAdd += 5;
            changed = true;
        }

        if (gamepad1.dpadDownWasReleased()) {
            yAdd -= 5;
            changed = true;
        }

        if (gamepad1.dpadRightWasReleased()) {
            xAdd += 5;
            changed = true;
        }

        if (gamepad1.dpadLeftWasReleased()) {
            xAdd -= 5;
            changed = true;
        }

        return changed;
    }

    /* ==================== PATH HELPER ==================== */

    public void gamepadStuff(double speed, Pose targetPose, double headingRad) {
        follower.setMaxPower(speed);
        follower.resumePathFollowing();

        Path path = new Path(new BezierLine(follower.getPose(), targetPose));
        path.setConstantHeadingInterpolation(headingRad);

        follower.followPath(path);
    }
}
