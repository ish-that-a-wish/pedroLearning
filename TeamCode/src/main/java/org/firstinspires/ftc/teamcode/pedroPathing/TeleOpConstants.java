package org.firstinspires.ftc.teamcode.pedroPathing;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.geometry.Pose;

@Config
public class TeleOpConstants {
    public static double HumanPlayerIntakePower = 0.75;
    public static double MoveToNearShootingPower = 1;
    public static double MoveToFarShootingPower = 1;
    public static double HumanPlayerIntakeHeading = Math.toRadians(180);
    public static double NearShotHeading = Math.toRadians(90);
    public static double FarShotHeading = Math.toRadians(90);
    public static Pose HumanPlayerPose = new Pose(20,10);
    public static Pose FarShotPose = new Pose(24, 24); //FIX IMPORTANT TO REAL FAR SHOT POSE WHICH WE TUNE
    public static Pose NearShotPose = new Pose(24,24);//FIX IMPORTANT TO REAL NEAR SHOT POSE WHICH WE TUNE
}
