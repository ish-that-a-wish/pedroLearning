package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@Autonomous
public class MoveSlide extends LinearOpMode {
    public static int ticks = 1000;
    public static double velocity = 0.8;
    public DcMotorEx horizontalSlideMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        horizontalSlideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        horizontalSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        horizontalSlideMotor.setTargetPosition(0);
        horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        horizontalSlideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        waitForStart();
        while (opModeIsActive()){
            horizontalSlideMotor.setTargetPosition(ticks);
            horizontalSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            horizontalSlideMotor.setPower(velocity);
        }
    }
}
