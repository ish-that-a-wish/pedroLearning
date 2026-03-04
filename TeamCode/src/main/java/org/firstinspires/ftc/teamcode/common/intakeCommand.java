package org.firstinspires.ftc.teamcode.common;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

public class intakeCommand extends CommandBase {

    private boolean startStop;
    private IntakeSubsystem intakeSubsystem;


    public intakeCommand(IntakeSubsystem intake, boolean startStop){

        this.startStop = startStop;
        intakeSubsystem = intake;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("INTAKE", "Initialized");
    }

    @Override
    public void execute(){
        super.execute();
        if (startStop)
            intakeSubsystem.runIntake();
        else
            intakeSubsystem.stopIntake();
        end(true);
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }

}
