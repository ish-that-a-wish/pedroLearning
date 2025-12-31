package org.firstinspires.ftc.teamcode.Constants;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//when tuning pidf tune f first and make sure you can hear the motors winding without them moving
//next tune p to make sure that it corrects fast enough without overcorrecting or undercorrecting
//change d last brings it slower to target point, high will move super slow, too low will move to fast
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
        .mass(12.565) // mass of robo
        .forwardZeroPowerAcceleration() // GET FROM FORWARD ZERO POWER ACCEL AUTO TUNER
        .lateralZeroPowerAcceleration() //GET FROM LATERAL ZERO POWER ACCEL AUTO TUNER
        .translationalPIDFCoefficients(new PIDFCoefficients()) //ADD COEFFECIENTS AFTER RUNING THE MANUAL TRANSLATIONAL TUNER
        .headingPIDFCoefficients(new PIDFCoefficients()) //ADD COEFFECIENTS AFTER RUNNING THE MANUAL HEADING PIDF TUNER
        .drivePIDFCoefficients(new FilteredPIDFCoefficients()) //TUNE BREAKING FIRST ADD COEfFECIENTS AFTER RUNNING MANUAL DRIVE TUNER, DONT CHANGE T VALUE USE THE DEFUALT
        .centripetalScaling(0.0005); //CAN TUNE USING CENTRIPETAL mANUAL TUNER TO FOLLOW CURVES BETTER 0.0005 IS DEFUALT
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightRear")
            .leftRearMotorName("leftRear")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity() //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
            .yVelocity(); //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.003064069755813305)
            .strafeTicksToInches(0.00304689192766577)
            .turnTicksToInches(0.002830092237130583)
            .leftPodY(5)
            .rightPodY(-5)
            .strafePodX(3.2)
            .leftEncoder_HardwareMapName("leftRear")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD);
    //TUNE BRAKING STRENGTH AND START BEFORE TUNING DRIVE PIDF
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            1,
            1
    ); //TUNE WITH DRIVE TUNER THE FORWARD AND BACK ONE
    // braking start where on path it does braking 0-1
    //braking strength is how much it breaks, more is more abrupt
    public static Follower CreateFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
