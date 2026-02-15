package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.pedropathing.paths.callbacks.PathCallback;

import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;
import java.util.stream.Collectors;

public class SpindexSubsystemReal extends SubsystemBase {
    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 = INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;
    public static boolean initMove;

    private boolean isTimerReset;

    private RobotHardware robotHardware;
    private enum spindexPoses {
        INTAKE_POSE_1,
        INTAKE_POSE_2,
        INTAKE_POSE_3,
        LAUNCH_POSE,

        LAUNCH_POSE_2,

        LAUNCH_POSE_3
    }
    private spindexPoses currentSpindexPose;
    public Motor intake;

    public SpindexSubsystemReal(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }


    public void shootArtifacts(){

    }

    public void intakeArtifacts(){
        if(!initMove){
            robotHardware.setSpindexPosition(INTAKE_POS_1);
            currentSpindexPose = spindexPoses.INTAKE_POSE_1; // current spindex pose 1
            initMove = true;
        }

        int nextIndex = getNextEmptySlotIndex();

        if (nextIndex < 0) {
            throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentSpindexPose == spindexPoses.INTAKE_POSE_1){ //if the current intake pose is 1
            Log.i("Polls", "Detected - Not busy");
            Log.i("Spindex Subsystem", "INTAKE 1 FILLED, MOVING TO INTAKE 2");
            robotHardware.setSpindexPosition(INTAKE_POS_2);
            currentSpindexPose = spindexPoses.INTAKE_POSE_2;
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentSpindexPose == spindexPoses.INTAKE_POSE_2){
            Log.i("Polls", "Detected - Not busy");
            Log.i("Spindex Subsystem", "INTAKE 2 FILLED, MOVING TO INTAKE 3");
            robotHardware.setSpindexPosition(INTAKE_POS_3);
            currentSpindexPose = spindexPoses.INTAKE_POSE_3;
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentSpindexPose == spindexPoses.INTAKE_POSE_3){
            Log.i("Polls", "Detected - Not busy");
            Log.i("Spindex Subsystem", "INTAKE 3 FILLED, MOVING TO LAUNCH POSE");
            robotHardware.setSpindexPosition(LAUNCH_POS_1);
            currentSpindexPose = spindexPoses.LAUNCH_POSE;
            initMove = false;
        }
    }

//    public void moveToShoot(){
//        if(!initMove){
//            robotHardware.setSpindexPosition(LAUNCH_POS_1);
//            currentIntakePose = spindexPoses.LAUNCH_POSE; // current spindex pose 1
//            initMove = true;
//        }
//
//        if(spindexFree() && currentIntakePose == spindexPoses.LAUNCH_POSE){
//
//
//            robotHardware.setSpindexPosition(LAUNCH_POS_2);
//            currentIntakePose = spindexPoses.LAUNCH_POSE_2;
//
//        }
//
//        if(spindexFree() && currentIntakePose == spindexPoses.LAUNCH_POSE_2){
//
//
//            robotHardware.setSpindexPosition(LAUNCH_POS_3);
//            currentIntakePose = spindexPoses.LAUNCH_POSE_3;
//
//        }
//
//        if(spindexFree() && currentIntakePose == spindexPoses.LAUNCH_POSE_3){
//
//
//            Log.i("SPINDEXER", "DONE WITH LAUNCHES");
//
//        }
//
//    }


    @Override
    public void periodic() {
        super.periodic();
//        if(currentIntakePose == spindexPoses.LAUNCH_POSE){
//            kickerSubsystem.moveKickerUpAndDown();
//        }
    }
    private int getNextEmptySlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.NONE)
                .collect(Collectors.toList());

        int nextEmptySlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            Log.i("ROBOT HARDWARE: ", String.valueOf(robotHardware));
            double currPos = robotHardware.getSpindexPosition();
            //this gets the closest empty slot to current position
            for (BallEntry entry : list) {
                if (Math.abs(entry.intakePosition - currPos) < distance) {
                    distance = Math.abs(entry.intakePosition - currPos);
                    nextEmptySlotIndex = entry.index;
                }
            }
        }
        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }
    public boolean spindexFree(){
        if(robotHardware.getSpindexPositionFromEncoder() > robotHardware.getSpindexPosition() - 0.05 && robotHardware.getSpindexPositionFromEncoder() < robotHardware.getSpindexPosition() + 0.05) {
            return true;
        }else{
            return false;
        }
    }
    public void init(){
        initMove = false;
        initializeWithEmpty();
        isTimerReset = false;
    }

    public void initializeWithEmpty() {
        storedColors.get(0).ballColor = GameColors.NONE;
        storedColors.get(1).ballColor = GameColors.NONE;
        storedColors.get(2).ballColor = GameColors.NONE;
    }
    public boolean isFull(){
        if(currentSpindexPose == spindexPoses.LAUNCH_POSE){
            Log.i("Spindex Subsystem", "FULL INDEX, ENDING COMMAND");
            return true;
        }
        else{
            Log.i("Spindex Subsystem", "INDEX NOT FULL RUNNING COMMAND");
            return false;
        }
    }
    public boolean isReadyToLaunch(){
        if(currentSpindexPose == spindexPoses.LAUNCH_POSE || currentSpindexPose == spindexPoses.LAUNCH_POSE_2 || currentSpindexPose == spindexPoses.LAUNCH_POSE_3){
            Log.i("SpindexSubsystem ", "Ready To Launch");
            return true;
        }
        else{
            Log.i("SpindexSubsytem ", "Not Ready To Launch");
            return false;
        }
    }
    public void spindexToLaunchPose(int launchNum){
        if(launchNum == 1){

        }
        else if (launchNum == 2) {

        }
        else if (launchNum == 3) {

        }
        else{
            // if the launch num is anything but 1 2 or 3
            Log.i("Spindex Subsystem", "Launch num is not valid");
        }
    }
}