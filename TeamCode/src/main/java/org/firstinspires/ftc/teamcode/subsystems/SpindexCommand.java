package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

public class SpindexCommand extends CommandBase {
    private RobotHardware robotHardware;
    private double position;
    private boolean initialized = false;
    private ElapsedTime actionDuration;
    public static double SPINDEX_POSITION_TOLERANCE = 0.08;

    private SpindexSubsystem spindex;


    public SpindexCommand(RobotHardware robotHardware, double position) {
        this.robotHardware = robotHardware;
        this.position = position;
        this.initialized = false;
        this.spindex = spindex;
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("INTAKE", "Initialized");
    }
    @Override
    public void execute(){
        super.execute();

//        if (!initialized) {
//            robotHardware.setSpindexPosition(position);
//            initialized = true;
//            actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//        }
//        double spindexPos = robotHardware.getSpindexPositionFromEncoder();
//
//        boolean retVal = (Math.abs(spindexPos - position) > SPINDEX_POSITION_TOLERANCE);
//        Log.i("SPINDEX ACTION", "POSITION: " + spindexPos + " Target: " + position);
//
//        if (!retVal) Log.i("SPINDEX ACTION", "Total Time Taken: " + actionDuration.milliseconds());

//        if(retVal) end(true);
    }
//    @Override
//public void execute(){
//    super.execute();
//    if (!initialized) {
//        robotHardware.setSpindexPosition(position);
//        initialized = true;
//        actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    }
//
//    double spindexPos = robotHardware.getSpindexPositionFromEncoder();
//    boolean retVal = (Math.abs(spindexPos - position) > SPINDEX_POSITION_TOLERANCE);
//
////            Log.i("SPINDEX ACTION", "POSITION: " + spindexPos + " Target: " + position);
//
//
//    if (!retVal)
//        Log.i("SPINDEX ACTION", "Total Time Taken: " + actionDuration.milliseconds());
//
//}
//    }
        @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }
}



//
//private RobotHardware robotHardware;
//private double position;
//private boolean initialized = false;
//private ElapsedTime actionDuration;
//public static double SPINDEX_POSITION_TOLERANCE = 0.08;
//
//public SpindexCommand(RobotHardware robotHardware, double position) {
//    this.robotHardware = robotHardware;
//    this.position = position;
//    this.initialized = false;
//}
//@Override
//public void initialize() {
//    super.initialize();
//    Log.i("INTAKE", "Initialized");
//}
//
//@Override
//public void execute(){
//    super.execute();
//    if (!initialized) {
//        robotHardware.setSpindexPosition(position);
//        initialized = true;
//        actionDuration = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
//    }
//
//    double spindexPos = robotHardware.getSpindexPositionFromEncoder();
//    boolean retVal = (Math.abs(spindexPos - position) > SPINDEX_POSITION_TOLERANCE);
//
////            Log.i("SPINDEX ACTION", "POSITION: " + spindexPos + " Target: " + position);
//
//
//    if (!retVal)
//        Log.i("SPINDEX ACTION", "Total Time Taken: " + actionDuration.milliseconds());
//
//}
//    }
//
//@Override
//public boolean isFinished(){
//    super.isFinished();
//    return true;
//}