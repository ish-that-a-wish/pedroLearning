package org.firstinspires.ftc.teamcode.Actions.Commands;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.intakeSubsystem;

public class intakeCommand extends CommandBase {

    private boolean startStop;
    private intakeSubsystem intakeSubsystem;


    public intakeCommand(intakeSubsystem intake, boolean startStop){

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
        if (startStop) {
//            Log.i("INTAKE COMMAND ", "RUNNING INTAKE");
            intakeSubsystem.runIntake();
        }
        else {
//            Log.i("INTAKE COMMAND", "STOPPING INTAKE");
            intakeSubsystem.stopIntake();
        }
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return true;
    }

}
