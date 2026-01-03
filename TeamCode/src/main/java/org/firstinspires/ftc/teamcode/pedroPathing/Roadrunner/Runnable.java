package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;
@Disabled
@Autonomous
public class Runnable extends LinearOpMode {
    private Pose initPose = new Pose(0,0, Math.toRadians(90));
    private Pose endPose = new Pose(0,10, Math.toRadians(90));
    private PathChain path1;
    private Follower follower;
    private DcMotorEx slideMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(initPose);

        buildPaths();
        waitForStart();
        follower.followPath(path1, 1, true);
        while(opModeIsActive()){
            telemetry.addData("Moving, currentPose, ", follower.getPose());
            follower.update();
            if(!follower.isBusy()){
                telemetry.addData("PATH COMPLETED, FINAL POSE: ", follower.getPose());
            }
            telemetry.update();
        }
    }
    public void buildPaths(){
        path1 = follower.pathBuilder()
                .addPath(new BezierLine(initPose, endPose))
                .addPoseCallback(new Pose(0,0), ()-> Actions.runBlocking(new MoveSlideAction(this.hardwareMap, 1000, true)), 0.5)
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();
    }

    public
    void
    moveSlide()
    {
        telemetry.addData("GOT TO 0,5 RUNNING MOTOR, CURRENT POSE: ", follower.getPose());
        slideMotor.setTargetPosition(1000);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(1);
    }
}
