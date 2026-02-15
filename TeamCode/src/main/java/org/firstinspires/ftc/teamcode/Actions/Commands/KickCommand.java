package org.firstinspires.ftc.teamcode.Actions.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

public class KickCommand extends CommandBase {
    private KickerSubsystem kicker;
    public KickCommand(KickerSubsystem kicker){
        this.kicker = kicker;
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("Kicker Command", "Initialized");
    }

    @Override
    public void execute(){
        super.execute();
        Log.i("Kicker Command ", "Running");
        kicker.moveKickerUpAndDown();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true; // only run the command once for now, add later
    }

}
