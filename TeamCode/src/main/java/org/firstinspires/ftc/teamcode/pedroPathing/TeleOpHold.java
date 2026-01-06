package org.firstinspires.ftc.teamcode.pedroPathing;

import android.util.Log;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
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
    private boolean followingPath = false;
    private DcMotorEx leftFront, rightFront, leftRear, rightRear;
    private double speed, strafe, turn;
    private Follower follower;
    private double xAdd = 0;
    private  double yAdd = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(new Pose(24,24,Math.toRadians(90)));
        initialize();

        waitForStart();

        while (opModeIsActive()) {
            follower.update();

            // SNAP TO TARGET (smooth joystick → path)
            if (gamepad1.bWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO SHOOTING POSE X: ", String.valueOf(BlueNearAutoConstants.SHOOTING_POSE.getX() + xAdd));
                Log.i("MOVING TO SHOOTING POSE Y: ", String.valueOf(BlueNearAutoConstants.SHOOTING_POSE.getY() + yAdd));
                follower.setMaxPower(1);
                follower.resumePathFollowing();
                follower.followPath(new Path(
                        new BezierLine(
                                follower.getPose(),
                                new Pose((24 + xAdd), (24 + yAdd))
                        )
                ));
                followingPath = true;
            }

            if (gamepad1.aWasReleased() && !follower.isBusy()) {
                Log.i("MOVING TO SHOOTING POSE X: ", String.valueOf(BlueFarConstants.SHOOTING_POSE.getX() + xAdd));
                Log.i("MOVING TO SHOOTING POSE Y: ", String.valueOf(BlueFarConstants.SHOOTING_POSE.getY() + yAdd));
                follower.setMaxPower(1);
                follower.resumePathFollowing();
                Path shootingPath = new Path(new BezierLine(
                        follower.getPose(),
                        new Pose((24 + xAdd), 24 + yAdd))
                );
                shootingPath.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(shootingPath);
                followingPath = true;
            }
            boolean poseChanged = moveThroughDpad();

            if (poseChanged && followingPath) {
                follower.setMaxPower(1);
                follower.resumePathFollowing();
                Path updatedPath = new Path(new BezierLine(
                        follower.getPose(),
                        new Pose(
                                24 + xAdd,
                                24 + yAdd
                        )
                ));
                updatedPath.setConstantHeadingInterpolation(Math.toRadians(90));
                follower.followPath(updatedPath);
            }


            // CANCEL PATH (driver panic button)
            if (gamepad1.rightBumperWasReleased()) {
                Log.i("FOLLOWING: ", "PAUSING PATH");
                follower.pausePathFollowing();
                follower.setMaxPower(0);
                followingPath = false;
            }

            // MANUAL DRIVE ONLY WHEN NOT FOLLOWING PATH
            if (!followingPath) {
                Log.i("FOLLOWING: ", "CONTROLLER MOVEMENT");
                speed  = -gamepad1.left_stick_y;
                strafe =  gamepad1.left_stick_x;
                turn   =  gamepad1.right_stick_x;
                moveWheels();
            }
        }
    }

    public void moveWheels() {
        double lf = speed + strafe + turn;
        double rf = speed - strafe - turn;
        double lr = speed - strafe + turn;
        double rr = speed + strafe - turn;

        // Normalize so no motor power goes past 1
        double max = Math.max(1.0,
                Math.max(Math.abs(lf),
                        Math.max(Math.abs(rf),
                                Math.max(Math.abs(lr), Math.abs(rr)))));

        lf /= max;
        rf /= max;
        lr /= max;
        rr /= max;

        leftFront.setPower(lf);
        rightFront.setPower(rf);
        leftRear.setPower(lr);
        rightRear.setPower(rr);
    }

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
    public boolean moveThroughDpad(){
        boolean changed = false;

        if(gamepad1.dpadUpWasReleased()){
            yAdd += 5;
            changed = true;
        }
        if(gamepad1.dpadDownWasReleased()){
            yAdd -= 5;
            changed = true;
        }
        if(gamepad1.dpadRightWasReleased()){
            xAdd += 5;
            changed = true;
        }
        if(gamepad1.dpadLeftWasReleased()){
            xAdd -= 5;
            changed = true;
        }

        return changed;
    }

}
