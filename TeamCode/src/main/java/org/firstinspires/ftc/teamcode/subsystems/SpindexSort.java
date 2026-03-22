package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.*;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.BallEntry;
import org.firstinspires.ftc.teamcode.common.GameColors;
import org.firstinspires.ftc.teamcode.common.RobotHardware;

import java.util.ArrayList;
import java.util.List;

@Config
public class SpindexSort extends SubsystemBase {

    // ---------------- POSITIONS ----------------
    public double DELTA = 0.375;
    public double INTAKE_1 = 0.195;
    public double INTAKE_2 = INTAKE_1 + DELTA;
    public double INTAKE_3 = INTAKE_2 + DELTA;
    public double LAUNCH_1 = INTAKE_1 + (DELTA * 1.5);
    public double LAUNCH_3 = LAUNCH_1 - DELTA;
    public double LAUNCH_2 = LAUNCH_3 - DELTA;

    // ---------------- CORE ----------------
    private final List<BallEntry> slots = new ArrayList<>();
    private final Telemetry telemetry;
    private final RobotHardware robot;
    public int currentIndex = 0;

    // ---------------- STATE ----------------
    private enum State { IDLE, INTAKING, FULL, LAUNCHING }
    private State state = State.IDLE;

    // ---------------- CONFIG ----------------
    public static long waitMs = 500;

    // ---------------- INPUT ----------------
    private boolean lastBeam = false;

    public SpindexSort(Telemetry telemetry, RobotHardware robot) {
        this.telemetry = telemetry;
        this.robot = robot;
        Log.i("Spindex", "Constructor called");
    }

    // ---------------- INIT ----------------
    public void init() {
        slots.clear();

        slots.add(new BallEntry(0, INTAKE_1, LAUNCH_1, GameColors.NONE));
        slots.add(new BallEntry(1, INTAKE_2, LAUNCH_2, GameColors.NONE));
        slots.add(new BallEntry(2, INTAKE_3, LAUNCH_3, GameColors.NONE));

        currentIndex = 0;
        state = State.IDLE;

        robot.setSpindexPosition(slots.get(0).intakePosition);

        Log.i("Spindex", "Initialized slots and moved to index 0");
    }

    // ---------------- UPDATE ----------------
    public void update() {
        logTelemetry();
        handleBeamBreak();
        Log.i("Spindex", "Update called, state = " + state);

        switch (state) {
            case IDLE:
                Log.i("Spindex", "Checking if not empty to start intaking");
                if (!isEmpty()) {
                    state = State.INTAKING;
                    Log.i("Spindex", "State changed to INTAKING");
                }
                break;

            case INTAKING:
                Log.i("Spindex", "INTAKING: isFull=" + isFull() + ", isBusy=" + robot.isSpindexBusy());
                if (!isFull() && !robot.isSpindexBusy()) {
                    Log.i("Spindex", "Scheduling move to closest empty slot");
                    schedule(moveToClosestEmptySlot());
                }
                if (isFull()) {
                    state = State.FULL;
                    Log.i("Spindex", "State changed to FULL");
                }
                break;

            case FULL:
                Log.i("Spindex", "FULL: starting launch sequence");
                schedule(startLaunchSequence());
                state = State.LAUNCHING;
                Log.i("Spindex", "State changed to LAUNCHING");
                break;

            case LAUNCHING:
                Log.i("Spindex", "LAUNCHING: waiting for launcher to call finishLaunch");
                break;
        }
    }

    // ---------------- BEAM BREAK ----------------
    private void handleBeamBreak() {
        boolean beam = robot.didBallDetectionBeamBreak();
        Log.i("Spindex", "Beam status: " + beam + ", lastBeam: " + lastBeam);

        if (beam && !lastBeam && !robot.isSpindexBusy()) {
            addColor(currentIndex, GameColors.UNKNOWN);
            Log.i("Spindex", "Beam detected, added UNKNOWN color at index " + currentIndex);
        }

        lastBeam = beam;
    }

    // ---------------- CLOSEST SLOT LOGIC ----------------
    private int getClosestSlotIndex(GameColors color, boolean intake) {
        double currPos = robot.getSpindexPosition();
        double bestDist = 5000;
        int bestIndex = -1;

        for (BallEntry entry : slots) {
            if (entry.ballColor == color) {
                double pos = intake ? entry.intakePosition : entry.launchPosition;
                double dist = Math.abs(pos - currPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = entry.index;
                    Log.i("Spindex", "New closest slot for " + color + ": " + bestIndex + ", dist=" + dist);
                }
            }
        }

        Log.i("Spindex", "Closest slot index for " + color + " is " + bestIndex);
        return bestIndex;
    }

    private int getClosestNonEmptyIndex() {
        double currPos = robot.getSpindexPosition();
        double bestDist = Double.MAX_VALUE;
        int bestIndex = -1;

        for (BallEntry entry : slots) {
            if (entry.ballColor != GameColors.NONE) {
                double dist = Math.abs(entry.launchPosition - currPos);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIndex = entry.index;
                    Log.i("Spindex", "New closest non-empty slot: " + bestIndex + ", dist=" + dist);
                }
            }
        }

        Log.i("Spindex", "Closest non-empty slot index: " + bestIndex);
        return bestIndex;
    }

    // ---------------- LAUNCH ----------------
    private Command startLaunchSequence() {
        Log.i("Spindex", "Start launch sequence scheduled");
        return new SequentialCommandGroup(
                new InstantCommand(() -> moveToPose(0, false)),
                new WaitCommand(waitMs),
                new InstantCommand(this::updateLaunchColors)
        );
    }

    public void finishLaunch() {
        setAll(GameColors.NONE);
        state = State.IDLE;
        Log.i("Spindex", "Finish launch: reset all slots and state to IDLE");
    }

    // ---------------- COLOR ----------------
    public void updateLaunchColors() {
        addColor(0, robot.getDetectedBallColorFromBackSensor());
        addColor(1, robot.getDetectedBallColorFromRightSensor());
        addColor(2, robot.getDetectedBallColorFromLeftSensor());

        Log.i("Spindex", "Colors updated from sensors: " +
                slots.get(0).ballColor + ", " +
                slots.get(1).ballColor + ", " +
                slots.get(2).ballColor);
    }

    // ---------------- MOVEMENT ----------------
    public void moveToPose(int index, boolean intake) {
        currentIndex = index;
        double pos = intake ? slots.get(index).intakePosition : slots.get(index).launchPosition;
        robot.setSpindexPosition(pos);

        Log.i("Spindex", "Moving to index " + index + " at position " + pos + (intake ? " (intake)" : " (launch)"));
    }

    // ---------------- SLOT STATE ----------------
    public boolean isFull() {
        boolean full = slots.stream().noneMatch(s -> s.ballColor == GameColors.NONE);
        Log.i("Spindex", "isFull = " + full);
        return full;
    }

    public boolean isEmpty() {
        boolean empty = slots.stream().allMatch(s -> s.ballColor == GameColors.NONE);
        Log.i("Spindex", "isEmpty = " + empty);
        return empty;
    }

    private void addColor(int index, GameColors color) {
        slots.get(index).ballColor = color;
        Log.i("Spindex", "addColor: " + color + " at index " + index);
    }

    private void removeBall(int index) {
        slots.get(index).ballColor = GameColors.NONE;
        Log.i("Spindex", "removeBall at index " + index);
    }

    private void setAll(GameColors color) {
        for (BallEntry s : slots) {
            s.ballColor = color;
        }
        Log.i("Spindex", "setAll slots to " + color);
    }

    // ---------------- COMMANDS ----------------
    private void schedule(Command cmd) {
        Log.i("Spindex", "Scheduling command: " + cmd);
        CommandScheduler.getInstance().schedule(cmd);
    }

    public Command moveToClosestEmptySlot() {
        int idx = getClosestSlotIndex(GameColors.NONE, true);
        Log.i("Spindex", "moveToClosestEmptySlot selected index: " + idx);

        if (idx == -1) return new InstantCommand();
        return new InstantCommand(() -> moveToPose(idx, true));
    }

    public Command moveToColor(GameColors color) {
        Log.i("Spindex", "moveToColor called for color: " + color);
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    int idx = getClosestSlotIndex(color, false);
                    if (idx == -1) {
                        idx = getClosestNonEmptyIndex();
                    }
                    currentIndex = idx;
                    if (idx != -1) {
                        moveToPose(idx, false);
                        Log.i("Spindex", "Moving to slot " + idx + " for color " + color);
                    }
                }),
                new WaitUntilCommand(() -> !robot.isSpindexBusy()),
                new InstantCommand(() -> {
                    if (currentIndex != -1) {
                        removeBall(currentIndex);
                        Log.i("Spindex", "Removed ball at " + currentIndex);
                    }
                })
        );
    }

    // ---------------- TELEMETRY ----------------
    private void logTelemetry() {
        telemetry.addData("State", state);
        telemetry.addData("Slots",
                "%s | %s | %s",
                slots.get(0).ballColor,
                slots.get(1).ballColor,
                slots.get(2).ballColor
        );
        telemetry.addData("Index", currentIndex);
        Log.i("Spindex", "Telemetry - State: " + state + ", Index: " + currentIndex +
                ", Slots: " + slots.get(0).ballColor + "|" + slots.get(1).ballColor + "|" + slots.get(2).ballColor);
    }
}