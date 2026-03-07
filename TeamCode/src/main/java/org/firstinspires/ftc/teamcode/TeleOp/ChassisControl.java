package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class ChassisControl {
    private DcMotorEx frontRightDriveMotor;
    private DcMotorEx frontLeftDriveMotor;
    private DcMotorEx backRightDriveMotor;
    private DcMotorEx backLeftDriveMotor;
    private Gamepad gamepad1;
    private Follower follower;
    private HardwareMap hardwareMap;
    public ChassisControl(Gamepad gamepad, Follower follower, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.gamepad1 = gamepad; // get the gamepad
        this.follower = follower;
    }
    public void update(){
        follower.update();

        followGamepadInputs();
    }
    public void followGamepadInputs(){

        double speed = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double turn = gamepad1.right_stick_x;

        double denominator = Math.max(Math.abs(speed) + Math.abs(strafe) + Math.abs(turn), 1);

        double frontLeftPower = (speed + turn + strafe) / denominator;
        double frontRightPower = (speed - turn - strafe) / denominator;
        double backLeftPower = (speed + turn - strafe) / denominator;
        double backRightPower = (speed - turn + strafe) / denominator;

        frontLeftDriveMotor.setPower(frontLeftPower);
        frontRightDriveMotor.setPower(frontRightPower);
        backLeftDriveMotor.setPower(backLeftPower);
        backRightDriveMotor.setPower(backRightPower);
    }
    public void init(){
        frontRightDriveMotor = hardwareMap.get(DcMotorEx.class, "rightFront");

        //this motor is oriented backwards, hence reversing direction
        frontLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "leftFront");
        frontLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        backRightDriveMotor = hardwareMap.get(DcMotorEx.class, "rightRear");

        //this motor is oriented backwards, hence reversing direction
        backLeftDriveMotor = hardwareMap.get(DcMotorEx.class, "leftRear");
        backLeftDriveMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}
