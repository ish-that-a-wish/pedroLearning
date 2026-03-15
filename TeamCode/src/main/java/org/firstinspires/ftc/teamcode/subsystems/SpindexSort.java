package org.firstinspires.ftc.teamcode.subsystems;

import android.app.Instrumentation;
import android.util.Log;

import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.Robot;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.stream.Collectors;

public class SpindexSort extends SubsystemBase {
    public double DELTA_BETWEEN_POSITIONS = 0.375;
    public double INTAKE_POS_1 = 0.195;
    public double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public double LAUNCH_POS_1 = INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;
    private List<BallEntry> slots = new ArrayList<>();
    private Telemetry telemetry;
    public int currentIndex;
    private RobotHardware robotHardware;
    private boolean hasMoveToLaunch = false;
    public SpindexSort(Telemetry telemetry, RobotHardware robotHardware){
        this.telemetry = telemetry;
        this.robotHardware = robotHardware;
    }
    public void init(){
        slots.add(new BallEntry(0, INTAKE_POS_1, LAUNCH_POS_1, GameColors.NONE));
        slots.add(new BallEntry(1, INTAKE_POS_2, LAUNCH_POS_2, GameColors.NONE));
        slots.add(new BallEntry(2, INTAKE_POS_3, LAUNCH_POS_3, GameColors.NONE));

        robotHardware.setSpindexPosition(slots.get(0).intakePosition);
        currentIndex = 0;
    }
    public void update(){
        telemetry.addData("Spindex Sorted ", "Current Game Colors: " + slots);
        telemetry.addData("Current Index: ", currentIndex);

        if(!isFull()) intake();
        if(isFull() && !hasMoveToLaunch){
            CommandScheduler.getInstance().schedule(
                    new InstantCommand(() -> moveToPose(0, false)),
                    new WaitCommand(150),
                    new InstantCommand(this::updateLaunchColors)
            );
        }
    }
    public boolean isFull(){return slots.stream().noneMatch(color -> color.ballColor == GameColors.NONE);}
    public boolean isEmpty(){return slots.stream().allMatch(color -> color.ballColor == GameColors.NONE);}
    public void moveToPose(int index, boolean intake){
        currentIndex = index;
        if(intake) robotHardware.setSpindexPosition(slots.get(currentIndex).intakePosition);
        else robotHardware.setSpindexPosition(slots.get(currentIndex).launchPosition);
    }
    public void intake(){
        List<BallEntry> emptySlots = slots.stream().filter(empty -> empty.ballColor == GameColors.NONE).collect(Collectors.toList());
        if(emptySlots.isEmpty()) return;
        List<Integer> emptyIndexes = new ArrayList<>();
        for(int i=0; i<emptySlots.size(); i++){
            emptyIndexes.add(emptySlots.get(i).index);
        }
        moveToPose(emptyIndexes.get(0), true);
        if(robotHardware.didBallDetectionBeamBreak()) addColor(currentIndex, GameColors.UNKNOWN);
    }

    public void addColor(int index, GameColors color){
        slots.get(index).ballColor = color;
    }
    public void removeBall(int index){
        slots.get(index).ballColor = GameColors.NONE;
    }
    public void updateLaunchColors(){
        GameColors slot0Color = robotHardware.getDetectedBallColorFromBackSensor();
        GameColors slot2Color = robotHardware.getDetectedBallColorFromLeftSensor();
        GameColors slot1Color = robotHardware.getDetectedBallColorFromRightSensor();

        addColor(0, slot0Color);
        addColor(1, slot1Color);
        addColor(2, slot2Color);
    }
}
