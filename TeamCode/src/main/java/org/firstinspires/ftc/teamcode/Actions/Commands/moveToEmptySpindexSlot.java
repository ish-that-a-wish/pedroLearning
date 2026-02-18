package org.firstinspires.ftc.teamcode.Actions.Commands;
import android.util.Log;
import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;

public class moveToEmptySpindexSlot extends CommandBase {
    private SpindexSubsystemReal spindex;
    public moveToEmptySpindexSlot(SpindexSubsystemReal spindex) {
        this.spindex = spindex;
        addRequirements(spindex);
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("Move To EmptySlot: ", "initalized");
    }

    @Override
    public void execute() {
        super.execute();

        spindex.initMove();

        spindex.intakeBalls();
    }

    @Override
    public boolean isFinished() {
        super.isFinished();
        return spindex.isReadyToLaunch();
    }
}