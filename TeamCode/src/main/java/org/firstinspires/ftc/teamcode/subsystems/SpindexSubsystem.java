package org.firstinspires.ftc.teamcode.subsystems;


import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.List;
import java.util.stream.Collectors;

//EXAMPLE CLASS FOR NOW WITHOUT PROPER INDEXING
public class SpindexSubsystem extends SubsystemBase {
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 = INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;
    private boolean initMove = false; // have a init move var so only able to init once
    public enum SpindexPoses {
        INTAKE_POSE_1,
        INTAKE_POSE_2,
        INTAKE_POSE_3,
        LAUNCH_POSE_1,

        LAUNCH_POSE_2,
        LAUNCH_POSE_3
    }
    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));
    private SpindexPoses currentPose;
    private RobotHardware robotHardware;
    public SpindexSubsystem(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }
        public boolean spindexFree(){
        return robotHardware.getSpindexPositionFromEncoder() > robotHardware.getSpindexPosition() - 0.05 && robotHardware.getSpindexPositionFromEncoder() < robotHardware.getSpindexPosition() + 0.05;
    }
    public void moveToPose(SpindexPoses pose){
        switch (pose){
            case INTAKE_POSE_1:
                robotHardware.setSpindexPosition(INTAKE_POS_1);
                currentPose = SpindexPoses.INTAKE_POSE_1;
                break;
            case INTAKE_POSE_2:
                robotHardware.setSpindexPosition(INTAKE_POS_2);
                currentPose = SpindexPoses.INTAKE_POSE_2;
                break;
            case INTAKE_POSE_3:
                robotHardware.setSpindexPosition(INTAKE_POS_3);
                currentPose = SpindexPoses.INTAKE_POSE_3;
                break;
            case LAUNCH_POSE_1:
                robotHardware.setSpindexPosition(LAUNCH_POS_1);
                currentPose = SpindexPoses.LAUNCH_POSE_1;
                break;
            case LAUNCH_POSE_2:
                robotHardware.setSpindexPosition(LAUNCH_POS_2);
                currentPose = SpindexPoses.LAUNCH_POSE_2;
                break;
            case LAUNCH_POSE_3:
                robotHardware.setSpindexPosition(LAUNCH_POS_3);
                currentPose = SpindexPoses.LAUNCH_POSE_3;
                break;
        }
    }
    public SpindexPoses getCurrentPose(){
        return currentPose;
    }
    // force init move just so that current pose is not null and no NPE
    public void initMove(){
        if(!initMove){
            moveToPose(SpindexPoses.INTAKE_POSE_1);
            initMove = true;
//            robotHardware.setSpindexPosition(INTAKE_POS_1);
//            initMove = true;
//            currentPose = SpindexPoses.INTAKE_POSE_1;
        }
    }
    //    public boolean isReadyToLaunch(){
//        return currentPose == SpindexPoses.LAUNCH_POSE_1; // return true if the current pose is launch pose
//    }
    public void intakeBalls(){
        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_1) moveToPose(SpindexPoses.INTAKE_POSE_2);

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_2) moveToPose(SpindexPoses.INTAKE_POSE_3);

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_3) moveToPose(SpindexPoses.LAUNCH_POSE_1);
    }
    public double convertSpindexPoseToDouble(SpindexPoses pose){
        switch (pose){
            case INTAKE_POSE_1:
                return INTAKE_POS_1;
            case INTAKE_POSE_2:
                return INTAKE_POS_2;
            case INTAKE_POSE_3:
                return INTAKE_POS_3;
            case LAUNCH_POSE_1:
                return LAUNCH_POS_1;
            case LAUNCH_POSE_2:
                return LAUNCH_POS_2;
            case LAUNCH_POSE_3:
                return LAUNCH_POS_3;
        }
        return robotHardware.getSpindexPositionFromEncoder(); // defualt to the current spindex pose
    }
    private int getNextEmptyIntakeSlotIndex() {
        // checks where the empty ball is
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.NONE)
                .collect(Collectors.toList());

        int nextEmptySlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition(); // get the current spindex pos

            //this gets the closest empty slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.intakePosition - currPos) < distance) {
                    distance = Math.abs(entry.intakePosition - currPos);
                    nextEmptySlotIndex = entry.index;
                }
            }
//            nextEmptySlotIndex = list.get(0).index;
        }

        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }
    public void moveToNextEmptyIntakeSlot(){
        int slotIndex = getNextEmptyIntakeSlotIndex();
        Log.i("SPINDEXER: ", "MOVING TO SLOT" + slotIndex);
        if(slotIndex == 0){
            moveToPose(SpindexPoses.INTAKE_POSE_1);
        }
        if(slotIndex == 1){
            moveToPose(SpindexPoses.INTAKE_POSE_2);
        }
        if(slotIndex == 2){
            moveToPose(SpindexPoses.INTAKE_POSE_3);
        }
        if(slotIndex == -1){
            return; // do nothing
        }
        return; // defualt to doing notbing
    }
    private int getNextFullLaunchSlotIndex() {
        // checks where the empty ball is
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor != GameColors.NONE)
                .collect(Collectors.toList());

        int nextEmptySlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition(); // get the current spindex pos

            //this gets the closest empty slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.launchPosition - currPos) < distance) {
                    distance = Math.abs(entry.launchPosition - currPos);
                    nextEmptySlotIndex = entry.index;
                }
            }
//            nextEmptySlotIndex = list.get(0).index;
        }

        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }
    public void moveToNextFullLaunchSlot(){
        int launchPos = getNextFullLaunchSlotIndex();
        if(launchPos == 0) moveToPose(SpindexPoses.LAUNCH_POSE_1);
        if(launchPos == 1) moveToPose(SpindexPoses.LAUNCH_POSE_2);
        if(launchPos == 2) moveToPose(SpindexPoses.LAUNCH_POSE_3);
        if(launchPos == -1) return; // defualt to do nun;
        return; //defualt to nun;
    }
    // init all of these with unkowns
    public void initalizeWithUnknowns(){
        storedColors.get(0).ballColor = GameColors.UNKNOWN;
        storedColors.get(1).ballColor = GameColors.UNKNOWN;
        storedColors.get(2).ballColor = GameColors.UNKNOWN;
    }
    public boolean isFull(){
        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }
    public boolean isEmpty(){
        return storedColors.stream().allMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }
    public void removeBallFromCurrentIndex(int index){
        storedColors.get(index).ballColor = GameColors.NONE;
    }
    public void addBallColorToIndex(int index, GameColors color){
        storedColors.get(index).ballColor = color;
    }
    public int getIndexFromPose(SpindexPoses pose, boolean intake){
        if(intake) {
            switch (pose) {
                case INTAKE_POSE_1:
                    return 0; // intake pose 1 corresponds to slot 0
                case INTAKE_POSE_2:
                    return 1;
                case INTAKE_POSE_3:
                    return 2;
            }
        }
        else{
            switch (pose){
                case LAUNCH_POSE_1:
                    return 0; // intake pose 1 corresponds to slot 0
                case LAUNCH_POSE_2:
                    return 1;
                case LAUNCH_POSE_3:
                    return 2;
            }
        }
        return -1; // not possible pose, everything else will fail make sure to add failsafe.
    }
    public SpindexPoses getPoseFromIndex(int index, boolean intake){
        if(intake){
            switch (index){
                case 0:
                    return SpindexPoses.INTAKE_POSE_1;
                case 1:
                    return SpindexPoses.INTAKE_POSE_2;
                case 2:
                    return SpindexPoses.INTAKE_POSE_3;
            }
        }
        else{
            switch (index) {
                case 0:
                    return SpindexPoses.LAUNCH_POSE_1;
                case 1:
                    return SpindexPoses.LAUNCH_POSE_2;
                case 2:
                    return SpindexPoses.LAUNCH_POSE_3;
            }
        }

        return null;
    }
    //just in case we have to init in moveToEmptySlot
    public GameColors getColorAtIndex(int index){
        return storedColors.get(index).ballColor;
    }
//    public void initalizeWithKnowns(GameColors color1, GameColors color2, GameColors color3){
//        storedColors.get(0).ballColor = color1;
//        storedColors.get(1).ballColor = color2;
//        storedColors.get(2).ballColor = color3;
//    }
    public void setCurrentPose(SpindexPoses pose){
        currentPose = pose;
    }
}
