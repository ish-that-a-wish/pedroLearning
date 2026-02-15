package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.NewActions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class KickerSubsystem extends SubsystemBase {
    private RobotHardware robotHardware;

    public static double LAUNCH_KICK_KICKING = 0.3;
    private SpindexSubsystemReal spindex;
    public static double LAUNCH_KICK_RESTING = 0.025;

    public KickerSubsystem(RobotHardware robotHardware, SpindexSubsystemReal spindex){
        this.spindex = spindex;
        this.robotHardware = robotHardware;
    }

    public void moveKickerUpAndDown(){
        if(spindex.isReadyToLaunch()) {
            Log.i("Kicker ", "Kicking ball");
            robotHardware.setLaunchKickPosition(LAUNCH_KICK_KICKING);
            robotHardware.setLaunchKickPosition(LAUNCH_KICK_RESTING);
        }
        else{
            Log.i("Kicker ", "Not kicking");
        }
//        robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_KICKING);
//        timer = new ElapsedTime(0);
//        timer.reset();
//
//        if(timer.time() >= 2000){
//            timer.reset();
//            while(timer.time() <= 2000){
//                robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_RESTING);
//            }
//        }

    }

    @Override
    public void periodic() {
        super.periodic();
        Log.i("INTAKE", "Called");
    }
}