//package org.firstinspires.ftc.teamcode.Actions.Commands;
//
//
//import android.util.Log;
//
//import com.arcrobotics.ftclib.command.CommandBase;
//import com.pedropathing.follower.Follower;
//import com.qualcomm.robotcore.hardware.Servo;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.teamcode.Actions.NewActions.LaunchKickAction;
//import org.firstinspires.ftc.teamcode.common.BallEntry;
//import org.firstinspires.ftc.teamcode.common.GameColors;
//import org.firstinspires.ftc.teamcode.common.RobotHardware;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
//import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;
//
//import java.util.List;
//import java.util.stream.Collectors;
//
//public class moveToEmptySpindexSlot extends CommandBase {
//    private SpindexSubsystemReal spindexSubsystemReal;
//    public moveToEmptySpindexSlot(SpindexSubsystemReal spindexSubsystemReal) {
//        this.spindexSubsystemReal = spindexSubsystemReal;
//    }
//
//    @Override
//    public void initialize() {
//        super.initialize();
//        Log.i("MOVE TO EMPTY SPINDEX SLOT", "Initialized");
//        spindexSubsystemReal.init();
//    }
//
//    @Override
//    public void execute() {
//        super.execute();
//
//        spindexSubsystemReal.intakeArtifacts();
//
//
////        if(!initMove){
////            robotHardware.setSpindexPosition(INTAKE_POS_1);
////            currentIntakePose = spindexPoses.INTAKE_POSE_1; // current spindex pose 1
////            initMove = true;
////        }
////
////        int nextIndex = getNextEmptySlotIndex();
////
////        if (nextIndex < 0) {
////            throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
////        }
////
////        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_1){ //if the current intake pose is 1
////            Log.i("Polls", "Detected - Not busy");
////            Log.i("moveToEmptySpindexSlot", "INTAKE 1 FILLED, MOVING TO INTAKE 2");
////            robotHardware.setSpindexPosition(INTAKE_POS_2);
////            currentIntakePose = spindexPoses.INTAKE_POSE_2;
////        }
////
////        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_2){
////            Log.i("Polls", "Detected - Not busy");
////            Log.i("moveToEmptySpindexSlot", "INTAKE 2 FILLED, MOVING TO INTAKE 3");
////            robotHardware.setSpindexPosition(INTAKE_POS_3);
////            currentIntakePose = spindexPoses.INTAKE_POSE_3;
////        }
////
////        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_3){
////            Log.i("Polls", "Detected - Not busy");
////            Log.i("moveToEmptySpindexSlot", "INTAKE 3 FILLED, MOVING TO LAUNCH POSE");
////            robotHardware.setSpindexPosition(LAUNCH_POS_1);
////            currentIntakePose = spindexPoses.LAUNCH_POSE;
////            follower.resumePathFollowing(); // resume the path
////        }
////
//////        if(currentIntakePose == spindexPoses.LAUNCH_POSE && spindexFree()){
//////            Log.i("Polls", "Detected - Not busy");
//////            Log.i("moveToEmptySpindexSlot", "INTAKE 3 FILLED, MOVING TO LAUNCH POSE");
//////
//////            robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_KICKING);
//////
//////            timer.reset();
//////            isTimerReset = true;
//////
////////            robotHardware.setSpindexPosition(LAUNCH_POS_1);
////////            currentIntakePose = spindexPoses.LAUNCH_POSE;
//////            follower.resumePathFollowing(); // resume the path
//////        }
//////
//////        if(timer.time() >= 2000 && isTimerReset){
//////            robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_RESTING);
//////            currentIntakePose = spindexPoses.LAUNCH_POSE_2;
//////        }
//
//    }
//
//    @Override
//    public boolean isFinished() {
//        super.isFinished();
//        return spindexSubsystemReal.isFull();
//    }
////
////    private int getNextEmptySlotIndex() {
////        List<BallEntry> list = storedColors.stream()
////                .filter(entry -> entry.ballColor == GameColors.NONE)
////                .collect(Collectors.toList());
////
////        int nextEmptySlotIndex = -1;
////
////        if (!list.isEmpty()) {
////            double distance = 5000;
////            Log.i("ROBOT HARDWARE: ", String.valueOf(robotHardware));
////            double currPos = robotHardware.getSpindexPosition();
////            //this gets the closest empty slot to current position
////            for (BallEntry entry : list) {
////                if (Math.abs(entry.intakePosition - currPos) < distance) {
////                    distance = Math.abs(entry.intakePosition - currPos);
////                    nextEmptySlotIndex = entry.index;
////                }
////            }
////        }
////        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
////        return nextEmptySlotIndex;
////    }
////
////    public boolean isFull(){
////
////        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
////
////    }
////
////    public void initializeWithEmpty() {
////        storedColors.get(0).ballColor = GameColors.NONE;
////        storedColors.get(1).ballColor = GameColors.NONE;
////        storedColors.get(2).ballColor = GameColors.NONE;
////    }
////
////    public boolean spindexFree(){
////        if(robotHardware.getSpindexPositionFromEncoder() > robotHardware.getSpindexPosition() - 0.05 && robotHardware.getSpindexPositionFromEncoder() < robotHardware.getSpindexPosition() + 0.05) {
////            return true;
////        }else{
////            return false;
////        }
////        }
//
//}
// }


package org.firstinspires.ftc.teamcode.Actions.Commands;

import static org.firstinspires.ftc.teamcode.AutoWithRR.FirstSpike.spindexFree;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Actions.NewActions.LaunchKickAction;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

import java.util.List;
import java.util.stream.Collectors;

public class moveToEmptySpindexSlot extends CommandBase {
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 = INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;
    public static boolean initMove;

    private boolean isTimerReset;

    private Follower follower;

    private RobotHardware robotHardware;
    private enum spindexPoses {
        INTAKE_POSE_1,
        INTAKE_POSE_2,
        INTAKE_POSE_3,
        LAUNCH_POSE,

        LAUNCH_POSE_2
    }
    private spindexPoses currentIntakePose;

    private ElapsedTime timer;

    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));
    public moveToEmptySpindexSlot(RobotHardware robotHardware, Follower follower) {
        this.follower = follower;
        this.robotHardware = robotHardware;
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("MOVE TO EMPTY SPINDEX SLOT", "Initialized");
        initMove = false;
        initializeWithEmpty();
        spindexFree = false;
        timer = new ElapsedTime(0);
        isTimerReset = false;

    }

    @Override
    public void execute() {
        super.execute();
        if(!initMove){
            robotHardware.setSpindexPosition(INTAKE_POS_1);
            currentIntakePose = spindexPoses.INTAKE_POSE_1; // current spindex pose 1
            initMove = true;
        }

        int nextIndex = getNextEmptySlotIndex();

        if (nextIndex < 0) {
            throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_1){ //if the current intake pose is 1
            Log.i("Polls", "Detected - Not busy");
            Log.i("moveToEmptySpindexSlot", "INTAKE 1 FILLED, MOVING TO INTAKE 2");
            robotHardware.setSpindexPosition(INTAKE_POS_2);
            currentIntakePose = spindexPoses.INTAKE_POSE_2;
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_2){
            Log.i("Polls", "Detected - Not busy");
            Log.i("moveToEmptySpindexSlot", "INTAKE 2 FILLED, MOVING TO INTAKE 3");
            robotHardware.setSpindexPosition(INTAKE_POS_3);
            currentIntakePose = spindexPoses.INTAKE_POSE_3;
        }

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentIntakePose == spindexPoses.INTAKE_POSE_3){
            Log.i("Polls", "Detected - Not busy");
            Log.i("moveToEmptySpindexSlot", "INTAKE 3 FILLED, MOVING TO LAUNCH POSE");
            robotHardware.setSpindexPosition(LAUNCH_POS_1);
            currentIntakePose = spindexPoses.LAUNCH_POSE;
            follower.resumePathFollowing(); // resume the path
        }

//        if(currentIntakePose == spindexPoses.LAUNCH_POSE && spindexFree()){
//            Log.i("Polls", "Detected - Not busy");
//            Log.i("moveToEmptySpindexSlot", "INTAKE 3 FILLED, MOVING TO LAUNCH POSE");
//
//            robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_KICKING);
//
//            timer.reset();
//            isTimerReset = true;
//
////            robotHardware.setSpindexPosition(LAUNCH_POS_1);
////            currentIntakePose = spindexPoses.LAUNCH_POSE;
//            follower.resumePathFollowing(); // resume the path
//        }
//
//        if(timer.time() >= 2000 && isTimerReset){
//            robotHardware.setLaunchKickPosition(LaunchKickAction.LAUNCH_KICK_RESTING);
//            currentIntakePose = spindexPoses.LAUNCH_POSE_2;
//        }

    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        if(currentIntakePose == spindexPoses.LAUNCH_POSE){
            Log.i("moveToEmptySpindexSlot", "FULL INDEX, ENDING COMMAND");
            Log.i("moveToEmptySpindexSlot", "IsFull: " + isFull());
            spindexFree = true;
            return isFull();
        }
        else{
            Log.i("moveToEmptySpindexSlot", "INDEX NOT FULL RUNNING COMMAND");
            return false;
        }
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

    public boolean isFull(){

        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);

    }

    public void initializeWithEmpty() {
        storedColors.get(0).ballColor = GameColors.NONE;
        storedColors.get(1).ballColor = GameColors.NONE;
        storedColors.get(2).ballColor = GameColors.NONE;
    }

    public boolean spindexFree(){
        if(robotHardware.getSpindexPositionFromEncoder() > robotHardware.getSpindexPosition() - 0.05 && robotHardware.getSpindexPositionFromEncoder() < robotHardware.getSpindexPosition() + 0.05) {
            return true;
        }else{
            return false;
        }
    }

}