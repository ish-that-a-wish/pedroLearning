package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.NullAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot;
import org.firstinspires.ftc.teamcode.Actions.NewActions.SpindexAction;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.spindexCommandReal;

import java.util.List;
import java.util.stream.Collectors;

@Config
public class SpindexSubsystem extends SubsystemBase {

    public static double SPINDEXER_INCREMENT = 0.1;
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 =  INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;

    private Servo spindexServo;

    boolean hasSeenBeamUnbroken = false;

    spindexCommandReal spindexCommandReal;

    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));

    private int previousIndex;
    public int currentIndex;
    private RobotHardware robotHardware;


    private boolean startupHandled = false;

    public SpindexSubsystem(RobotHardware robotHardware, Follower follower){
        this.previousIndex = -1;
        this.currentIndex = -1; //to ensure the first move happens
        this.robotHardware = robotHardware;
//        moveToEmptySlot = new moveToEmptySpindexSlot(this.robotHardware, follower);
    }

    private SpindexSubsystem() {}

//    public void moveToNextEmptySlotCommand(){
//        int nextIndex = getNextEmptySlotIndex();
//        if (nextIndex < 0) throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
//
//        previousIndex = currentIndex;
//        currentIndex = nextIndex;
//
//        Log.i("SPINDEXER", "Moving to Next Intake Slot");
//        Log.i("SPINDEXER, ", "CURRENT POS: " + (robotHardware.getSpindexPosition()));
//        Log.i("SPINDEXER", "NEXT POS: " + storedColors.get(currentIndex).intakePosition);
//
//        if(nextIndex == 0){
//            robotHardware.setSpindexPosition(INTAKE_POS_1);
//
//        } else if (nextIndex == 1) {
//            robotHardware.setSpindexPosition(INTAKE_POS_2);
//
//        }else {
//            robotHardware.setSpindexPosition(INTAKE_POS_3);
//
//        }
//
//
//        //return new SpindexCommand(robotHardware, storedColors.get(currentIndex).intakePosition);
//    }

    private boolean lastBeamState = false;
//    @Override
//    public void periodic() {
//        if(robotHardware.didBallDetectionBeamBreak()){
//            Log.i("SPINDEX SUBSYS ", "BEAMBREAK BROKE");
//            CommandScheduler.getInstance().schedule(moveToEmptySlot);
//        }
//        super.periodic();
//    }
    public boolean tryMoveToNextEmptySlot() {

        if (!robotHardware.didBallDetectionBeamBreak()) {
            return false; // no ball present
        }

        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) return false;

        if (nextIndex == currentIndex) return false;

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        robotHardware.setSpindexPosition(
                storedColors.get(currentIndex).intakePosition
        );

        Log.i("SPINDEXER", "Indexed to slot " + currentIndex);
        Log.i("SPINDEXER", "Beam: " + robotHardware.didBallDetectionBeamBreak());
        return true;
    }


    public void moveToNextEmptySlotCommand() {

        Log.i("SPINDEXER", "moveToNextEmptySlotCommand called | Beam: " + robotHardware.didBallDetectionBeamBreak() +
                " | currentIndex: " + currentIndex);

        boolean beamBroken = robotHardware.didBallDetectionBeamBreak();

        Log.i("SPINDEXER",
                "Beam=" + beamBroken +
                        " lastBeam=" + lastBeamState +
                        " startupHandled=" + startupHandled);

        // Handle startup ball
        if (!startupHandled && beamBroken) {
            startupHandled = true;
        }
        // Normal rising edge
        else if (!(beamBroken && !lastBeamState)) {
            lastBeamState = beamBroken;
            return;
        }

        lastBeamState = beamBroken;

        int nextIndex = getNextEmptySlotIndex();
        if (nextIndex < 0) {
            Log.w("SPINDEXER", "No empty slots available");
            return;
        }

        if (nextIndex == currentIndex) return;

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        robotHardware.setSpindexPosition(
                storedColors.get(currentIndex).intakePosition
        );
    }
    public void initializeWithPPG() {
        storedColors.get(0).ballColor = GameColors.PURPLE;
        storedColors.get(1).ballColor = GameColors.PURPLE;
        storedColors.get(2).ballColor = GameColors.GREEN;
    }

    public void initializeWithEmpty() {
        storedColors.get(0).ballColor = GameColors.NONE;
        storedColors.get(1).ballColor = GameColors.NONE;
        storedColors.get(2).ballColor = GameColors.NONE;
    }

    public void setCurrentIndex(int index) {
        this.previousIndex = this.currentIndex;
        this.currentIndex = index;
    }


    public void moveSpindex(){

    }




    public void initializeWithUnknowns() {
        storedColors.get(0).ballColor = GameColors.UNKNOWN;
        storedColors.get(1).ballColor = GameColors.UNKNOWN;
        storedColors.get(2).ballColor = GameColors.UNKNOWN;
    }

//    private int getNextEmptySlotIndex() {
//        List<BallEntry> list = storedColors.stream()
//                .filter(entry -> entry.ballColor == GameColors.NONE)
//                .collect(Collectors.toList());
//
//        int nextEmptySlotIndex = -1;
//
//        if (!list.isEmpty()) {
//            double distance = 5000;
//            double currPos = robotHardware.getSpindexPosition();
//
//            //this gets the closest empty slot to current position
//            for (BallEntry entry: list) {
//                if (Math.abs(entry.intakePosition - currPos) < distance) {
//                    distance = Math.abs(entry.intakePosition - currPos);
//                    nextEmptySlotIndex = entry.index;
//                }
//            }
////            nextEmptySlotIndex = list.get(0).index;
//        }
//
//        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
//        return nextEmptySlotIndex;
//    }

    private int getNextEmptySlotIndex() {

        double currPos = robotHardware.getSpindexPosition();
        double closestDistance = Double.MAX_VALUE;
        int nextEmptySlotIndex = -1;

        for (BallEntry entry : storedColors) {

            if (entry.ballColor != GameColors.NONE) continue;

            double distance = Math.abs(entry.intakePosition - currPos);

            if (distance < closestDistance) {
                closestDistance = distance;
                nextEmptySlotIndex = entry.index;
            }
        }

        Log.i("SPINDEXER", "Next empty slot index: " + nextEmptySlotIndex);
        return nextEmptySlotIndex;
    }

    private int getNextFullSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor != GameColors.NONE)
                .collect(Collectors.toList());

        int nextFullSlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition();

            //this gets the closest full slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.launchPosition - currPos) < distance) {
                    distance = Math.abs(entry.launchPosition - currPos);
                    nextFullSlotIndex = entry.index;
                }
            }
//            nextFullSlotIndex = list.get(0).index;
        }

//        Log.i("SPINDEXER", "NEXT FULL SLOT INDEX: " + nextFullSlotIndex);
        return nextFullSlotIndex;
    }

    public int getNextGreenSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.GREEN)
                .collect(Collectors.toList());

        int nextGreenSlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition();

            //this gets the closest full slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.launchPosition - currPos) < distance) {
                    distance = Math.abs(entry.launchPosition - currPos);
                    nextGreenSlotIndex = entry.index;
                }
            }
//            nextGreenSlotIndex = list.get(0).index;
        }

//        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextGreenSlotIndex);
        return nextGreenSlotIndex;
    }

    public int getNextPurpleSlotIndex() {
        List<BallEntry> list = storedColors.stream()
                .filter(entry -> entry.ballColor == GameColors.PURPLE)
                .collect(Collectors.toList());

        int nextPurpleSlotIndex = -1;

        if (!list.isEmpty()) {
            double distance = 5000;
            double currPos = robotHardware.getSpindexPosition();

            //this gets the closest full slot to current position
            for (BallEntry entry: list) {
                if (Math.abs(entry.launchPosition - currPos) < distance) {
                    distance = Math.abs(entry.launchPosition - currPos);
                    nextPurpleSlotIndex = entry.index;
                }
            }

//            nextPurpleSlotIndex = list.get(0).index;
        }

//        Log.i("SPINDEXER", "NEXT GREEN SLOT INDEX: " + nextPurpleSlotIndex);
        return nextPurpleSlotIndex;
    }

    public Command moveToSlotZeroLaunchPosition() {
        previousIndex = currentIndex;
        currentIndex = 0;

        return new SpindexCommand(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Command moveToNextFullSlotCommand() {
        int nextIndex = getNextFullSlotIndex();
        if (nextIndex < 0 ) throw new ArrayIndexOutOfBoundsException("NEXT SPINDEX LESS THAN 0");

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        return new SpindexCommand(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Command moveToNextGreenSlotCommand() {
        int nextIndex = getNextGreenSlotIndex();
        if (nextIndex < 0) throw new ArrayIndexOutOfBoundsException("NEXT SPINDEX LESS THAN 0");

        previousIndex = currentIndex;
        currentIndex = nextIndex;
        return new SpindexCommand(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public Command moveToNextPurpleSlotCommand() {
        int nextIndex = getNextPurpleSlotIndex();
        if (nextIndex < 0) throw new ArrayIndexOutOfBoundsException("NEXT SPINDEX LESS THAN 0");

        previousIndex = currentIndex;
        currentIndex = nextIndex;

        return new SpindexCommand(robotHardware, storedColors.get(currentIndex).launchPosition);
    }

    public boolean isReadyForIntake() {
        boolean retVal = false;

        //this is intentionally NOT using the encoder position
        //for a servo, the getposition returns the last value passed into setposition
        //this means we gave the spindexer a command to move to an intake position
        double currentPos = robotHardware.getSpindexPosition();
//        Log.i("SPINDEXER", "isReadyForIntake: currentPos: " + currentPos);

        //this should be the intake position for current index
        if (Math.abs(storedColors.get(currentIndex).intakePosition - currentPos) < SpindexCommand.SPINDEX_POSITION_TOLERANCE && storedColors.get(currentIndex).ballColor == GameColors.NONE) {
            Log.i("SPINDEXER", "isReadyForIntake: we are or will be at an intake position: ");
            retVal = true;
        }
        else{
            Log.i("SPINDEXER", "Not ready for intake pos or not at intake pos");
            if(!(Math.abs(storedColors.get(currentIndex).intakePosition - currentPos) < SpindexCommand.SPINDEX_POSITION_TOLERANCE)){
                Log.i("SPINDEXER, ", "IS NOT AT INTAKE POS");
            }
            if(!(storedColors.get(currentIndex).ballColor == GameColors.NONE)){
                Log.i("SPINDEXER", "NOT AT EMPTY SLOT");
                CommandScheduler.getInstance().schedule(spindexCommandReal);
            }
        }

        Log.i("SPINDEXER", "isReadyForIntake: " + retVal);

        retVal = retVal && !robotHardware.isSpindexBusy() && !isFull();

        Log.i("SPINDEXER", "isReadyForIntake: " + retVal);

        //spindex should not be busy moving and should be at an intake position
        return retVal;
    }



    public boolean isFull() {
        Log.i("SPINDEX SUBSYSTEM: ", "IS FULL: " + storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE));
        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor == GameColors.NONE);
    }

    public int fullSlotCount() {initializeWithEmpty();
        return (int) storedColors.stream().filter(ballEntry -> ballEntry.ballColor != GameColors.NONE).count();
    }

    public boolean isEmpty() {
        return storedColors.stream().noneMatch(ballEntry -> ballEntry.ballColor != GameColors.NONE);
    }

    public void updateBallColorAtCurrentIndex(GameColors color) {
//        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(currentIndex).ballColor = color;
    }

    public void updateBallColorAtIndex(int index, GameColors color) {
//        Log.i("SPINDEXER", "STORE CURRENT BALL: " + color);
        storedColors.get(index).ballColor = color;
    }

    public void clearBallAtCurrentIndex() {

//        Log.i("SPINDEXER", "CLEAR CURRENT BALL");
        storedColors.get(currentIndex).ballColor = GameColors.NONE;
    }

    public void clearBallAtIndex(int index) {

//        Log.i("SPINDEXER", "CLEAR BALL AT INDEX");
        storedColors.get(index).ballColor = GameColors.NONE;
    }

}
