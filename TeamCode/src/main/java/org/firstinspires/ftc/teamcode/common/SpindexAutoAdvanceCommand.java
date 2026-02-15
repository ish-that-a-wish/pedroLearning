package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Spindex;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystem;

public class SpindexAutoAdvanceCommand extends CommandBase {

    private final SpindexSubsystem spindex;
    private final double timeoutSeconds;
    private double startTime;

    private Spindex rrSpindex;

    RobotHardware robotHardware;

    public SpindexAutoAdvanceCommand(
            SpindexSubsystem spindex,
            double timeoutSeconds,
            RobotHardware robotHardware,
            Spindex spindexrr
    ) {
        this.spindex = spindex;
        this.timeoutSeconds = timeoutSeconds;
        this.robotHardware = robotHardware;
        this.rrSpindex = spindexrr;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis() / 1000.0;
    }

    @Override
    public void execute() {
        //spindex.moveToNextEmptySlotCommand();
        Actions.runBlocking(rrSpindex.moveToNextEmptySlotAction());
        Log.i("SPINDEX STATUS", "Running");
    }

    @Override
    public boolean isFinished() {
        boolean timedOut =
                (System.currentTimeMillis() / 1000.0) - startTime >= timeoutSeconds;

        return rrSpindex.isFull() || timedOut;
    }

    @Override
    public void end(boolean interrupted) {
        // Optional: stop intake or log
    }
}
