package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

@TeleOp
public class AutoPark extends LinearOpMode {
    public Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(new Pose(0,0, Math.toRadians(180))); //wherever auto ends
        waitForStart();
        while(opModeIsActive()){
            follower.update();
            telemetry.addData("CURRENT POSE: ", follower.getPose());
            telemetry.update();
        }
    }
}
