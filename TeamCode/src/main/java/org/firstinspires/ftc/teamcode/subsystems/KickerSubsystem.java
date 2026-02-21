package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class KickerSubsystem extends SubsystemBase {
    private RobotHardware robotHardware;

    public static double LAUNCH_KICK_KICKING = 0.3;
    public static double LAUNCH_KICK_RESTING = 0.025;
    public static double KICKING_TOLERANCE = 0.05;

    public KickerSubsystem(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }

    public void moveKickerUp(double SpindexKickingPose){
        if(Math.abs(robotHardware.getSpindexPosition() - SpindexKickingPose) < KICKING_TOLERANCE) {
            Log.i("Kicker ", "Kicking ball up");
            robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING);
//            robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);
        }
        else{
            Log.i("Kicker ", "Not kicking");
        }
    }
    public void moveKickerDown(double SpindexKickingPose){
        if(Math.abs(robotHardware.getSpindexPosition() - SpindexKickingPose) < KICKING_TOLERANCE) {
            Log.i("Kicker ", "Kicking ball down");
            robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);
//            robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);
        }
        else{
            Log.i("Kicker ", "Not kicking");
        }
    }

    @Override
    public void periodic() {
        super.periodic();
        Log.i("INTAKE", "Called");
    }
}