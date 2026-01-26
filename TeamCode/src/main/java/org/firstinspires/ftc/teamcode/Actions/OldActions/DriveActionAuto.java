package org.firstinspires.ftc.teamcode.Actions.OldActions;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
@Disabled
@Autonomous
public class DriveActionAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            Actions.runBlocking(
                    new DriveAction(new Pose(0,0), new Pose(0,10), this.hardwareMap)
            );
        }
    }
}
