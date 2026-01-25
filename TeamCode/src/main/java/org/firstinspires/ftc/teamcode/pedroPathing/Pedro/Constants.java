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
            .mass(15.876) // mass of robo
            .forwardZeroPowerAcceleration(-25.464873123258485) // GET FROM FORWARD ZERO POWER ACCEL AUTO TUNER
            .lateralZeroPowerAcceleration(-64.86831545428669) //GET FROM LATERAL ZERO POWER ACCEL AUTO TUNER
            .translationalPIDFCoefficients(new PIDFCoefficients(0.45,0,0.025,0.0223)) //ADD COEFFECIENTS AFTER RUNING THE MANUAL TRANSLATIONAL TUNER
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0, 0.03, 0.025)) //ADD COEFFECIENTS AFTER RUNNING THE MANUAL HEADING PIDF TUNER
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.018, 0.005, 0.0, 0.6, 0.05)) //TUNE BREAKING FIRST ADD COEfFECIENTS AFTER RUNNING MANUAL DRIVE TUNER, DONT CHANGE T VALUE USE THE DEFUALT
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
            .xVelocity(65.83987989287641) //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
            .yVelocity(53.11009999682826); //ADD LATERAL VELOCITY AUTO TUNING FILES RETURNED VALUE
    public static ThreeWheelConstants localizerConstants = new ThreeWheelConstants()
            .forwardTicksToInches(0.0019937428105135557)
            .strafeTicksToInches(.001989436789)
            .turnTicksToInches(-0.0019990825241169067)
            .leftPodY(-7.25)
            .rightPodY(7.25)
            .strafePodX(0)
            .leftEncoder_HardwareMapName("leftRear")
            .rightEncoder_HardwareMapName("leftFront")
            .strafeEncoder_HardwareMapName("rightRear")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.REVERSE);
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