package org.firstinspires.ftc.teamcode.pedroPathing.Pedro;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

//when tuning pidf tune f first and make sure you can hear the motors winding without them moving
//next tune p to make sure that it corrects fast enough without overcorrecting or undercorrecting
//change d last brings it slower to target point, high will move super slow, too low will move to fast
@Config
public class Constants {
    public static double breakingStrength = 0.4;
    public static double breakingStart = 0.9;
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.565) // mass of robo
            .forwardZeroPowerAcceleration(-32.96760734485633) // GET FROM FORWARD ZERO POWER ACCEL AUTO TUNER
            .lateralZeroPowerAcceleration(-74.447627183273075) //GET FROM LATERAL ZERO POWER ACCEL AUTO TUNER
            .translationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0.01,0.012)) //ADD COEFFECIENTS AFTER RUNING THE MANUAL TRANSLATIONAL TUNER
            .headingPIDFCoefficients(new PIDFCoefficients(0.56, 0.01, 0.02, 0.02467)) //ADD COEFFECIENTS AFTER RUNNING THE MANUAL HEADING PIDF TUNER
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.002, 0.03, 0.00002, 0.6, 0)) //TUNE BREAKING FIRST ADD COEfFECIENTS AFTER RUNNING MANUAL DRIVE TUNER, DONT CHANGE T VALUE USE THE DEFUALT
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
            .xVelocity(57.256939617181067) //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
            .yVelocity(42.836246358680067); //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.003064069755813305)
            .strafeTicksToInches(0.00304689192766577)
            .turnTicksToInches(0.0028300922371330583)
            .leftPodY(5)
            .rightPodY(-5)
            .strafePodX(3.2)
            .leftEncoder_HardwareMapName("leftRear")
            .rightEncoder_HardwareMapName("rightRear")
            .strafeEncoder_HardwareMapName("rightFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.FORWARD);
    //TUNE BRAKING STRENGTH AND START BEFORE TUNING DRIVE PIDF
    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            100,
            breakingStrength,
            breakingStart
    ); //TUNE WITH DRIVE TUNER THE FORWARD AND BACK ONE
    // braking start where on path it does braking 0-1
    //braking strength is how much it breaks, more is more abrupt
    public static Follower createFollower(HardwareMap hardwareMap){
        return new FollowerBuilder(followerConstants, hardwareMap)
                .threeWheelLocalizer(localizerConstants)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}