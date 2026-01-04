package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Pedro.Constants;

@Autonomous
@Disabled
@Config
public class TestAuto extends LinearOpMode {
    public static int ticks = 100;
    private Follower follower;
    @Override
    public void runOpMode() throws InterruptedException {
        follower = Constants.createFollower(this.hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            Actions.runBlocking(
                    new ParallelAction(
                            new MoveSlideAction(this.hardwareMap, ticks, true)
                    )
            );
        }
    }
}
