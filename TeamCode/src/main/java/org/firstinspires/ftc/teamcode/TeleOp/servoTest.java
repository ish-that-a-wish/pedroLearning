package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

@TeleOp
@Config
public class servoTest extends LinearOpMode {
    private RobotHardware robot;
    public static double pos = 0.01;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new RobotHardware(this.hardwareMap);
        waitForStart();
        while(opModeIsActive()){
            if(gamepad1.a) {
                robot.setLaunchVisorPosition(pos);
            }
        }
    }
}
