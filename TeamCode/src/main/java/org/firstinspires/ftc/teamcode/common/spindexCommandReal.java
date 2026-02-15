package org.firstinspires.ftc.teamcode.common;


import android.util.Log;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AutoWithRR.BlueNearAuto;
import org.firstinspires.ftc.teamcode.Near.BlueNearAutoConstants;
import org.firstinspires.ftc.teamcode.subsystems.SpindexCommand;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;

import java.util.List;
import java.util.stream.Collectors;

public class spindexCommandReal extends CommandBase {
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 =  INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;

    private RobotHardware robotHardware;

    private SpindexSubsystem spindexSubsystem;

    private Follower follower;

    private ElapsedTime timer;

    enum State {
        FOLLOWING_PATH,
        WIGGLING,
        RESUMING
    }

    private State state = State.FOLLOWING_PATH;

    private final ElapsedTime wiggleTimer = new ElapsedTime();

    private static final double WIGGLE_TIME = 1.5; // seconds
    private static final double POSE_TOLERANCE = 1.0; // inches

    private boolean interruptedOnce = false;


    public List<BallEntry> storedColors = List.of(
            new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE),
            new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE),
            new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));


    public spindexCommandReal(SpindexSubsystem spindex, RobotHardware robotHardware, Follower follower){
        this.follower = follower;
        this.robotHardware = robotHardware;
        this.spindexSubsystem = spindex;
        addRequirements(spindexSubsystem);
    }

    public spindexCommandReal(SpindexSubsystem spindex, RobotHardware robotHardware){
        this.follower = follower;
        this.robotHardware = robotHardware;
        this.spindexSubsystem = spindex;
        addRequirements(spindexSubsystem);
    }

    private void runWiggle() {
        double power = Math.sin(wiggleTimer.seconds() * 8) > 0 ? 0.3 : -0.3;
        follower.setMaxPower(0.5);
     }


    private boolean atPose(Pose target) {
        return follower.getPose().distanceFrom(target) < POSE_TOLERANCE;
    }



    @Override
    public void initialize() {
        super.initialize();
        Log.i("SPINDEX", "Initialized");
    }

    @Override
    public void execute(){
        super.execute();
        spindexSubsystem.moveToNextEmptySlotCommand();

        switch (state){
            case FOLLOWING_PATH:
                if(!interruptedOnce && follower.getPose().distanceFrom(BlueNearAutoConstants.FIRST_SPIKE) < POSE_TOLERANCE){

                    interruptedOnce = true;

                    follower.pausePathFollowing();
                    wiggleTimer.reset();

                    state = State.WIGGLING;
                    break;

                }


            case WIGGLING:
                runWiggle();

                if (wiggleTimer.seconds() >= WIGGLE_TIME || spindexSubsystem.isFull()) {
                    follower.resumePathFollowing();
                    follower.followPath(BlueNearAutoConstants.moveToShootSpike1);
                    state = State.RESUMING;
                    break;
                }


            case RESUMING:

                if (!follower.isBusy()) {
                    follower.resumePathFollowing(); // failsafe
                }

                break;

        }


//        follower.setMaxPower(0.5);
//        spindexSubsystem.moveToNextEmptySlotCommand();
//        if(spindexSubsystem.isFull()){
//
//        }

//        Log.i("SPINDEX COMMAND REAL: ", "NEXT EMPTY SLOT: " + getNextEmptySlotIndex());
////        follower.pausePathFollowing();
//        follower.setMaxPower(0.5);
//        timer = new ElapsedTime(0);
//
//        while (timer.time() < 5000){
//            int nextIndex = getNextEmptySlotIndex();
//            if (nextIndex < 0) throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
//            robotHardware.setSpindexPosition(storedColors.get(nextIndex).intakePosition);
////        robotHardware.setSpindexPosition(SpindexSubsystem.INTAKE_POS_2);
//        }
//
//        follower.resumePathFollowing();
//        end(true);

    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
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
//            Log.i("SPINDEX COMMAND REAL: ", "CURR POS" + currPos);
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
////        Log.i("SPINDEXER", "NEXT EMPTY SLOT INDEX: " + nextEmptySlotIndex);
//        return nextEmptySlotIndex;
//    }
//    public Command moveToNextEmptySlotCommand(){
//        int nextIndex = getNextEmptySlotIndex();
//        if (nextIndex < 0) throw new ArrayIndexOutOfBoundsException("SPINDEX NEXT INDEX LESS THAN 0");
//
//
//        Log.i("SPINDEXER", "Moving to Next Intake Slot");
//        Log.i("SPINDEXER, ", "CURRENT POS: " + (robotHardware.getSpindexPosition()));
////        Log.i("SPINDEXER", "NEXT POS: " + storedColors.get(currentIndex).intakePosition);
//        return new SpindexCommand(robotHardware, storedColors.get(currentIndex).intakePosition);
//    }
}
