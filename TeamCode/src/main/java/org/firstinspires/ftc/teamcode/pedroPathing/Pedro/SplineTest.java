package org.firstinspires.ftc.teamcode.pedroPathing.Pedro;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class SplineTest extends LinearOpMode {
    private Pose INIT_POSE = new Pose(56, 8);
    private Pose END_POSE = new Pose(84, 36);
    private Pose Control_Pose = new Pose(53.64, 37.17340286831812);
    private PathChain spline;
    public Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        follower.setStartingPose(INIT_POSE);
        buildPaths();
        waitForStart();
        while(opModeIsActive()){
            follower.update();
            follower.followPath(spline);
            telemetry.addData("Current Pose: ", follower.getPose());
        }
    }
    public void buildPaths(){
        spline = follower.pathBuilder()
                .addPath(new BezierCurve(INIT_POSE, Control_Pose, END_POSE))
                .build();
    }
}
