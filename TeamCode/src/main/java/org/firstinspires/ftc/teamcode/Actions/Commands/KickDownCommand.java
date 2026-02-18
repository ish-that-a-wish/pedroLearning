package org.firstinspires.ftc.teamcode.Actions.Commands;

import static org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot.LAUNCH_POS_1;
import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.KICKING_TOLERANCE;

import android.util.Log;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;

public class KickDownCommand extends CommandBase {
    private KickerSubsystem kicker;
    private RobotHardware robotHardware;
    public KickDownCommand(KickerSubsystem kicker, RobotHardware robotHardware){
        this.robotHardware = robotHardware;
        this.kicker = kicker;
    }

    @Override
    public void initialize() {
        super.initialize();
        Log.i("Kicker Down Command", "Initialized");
    }

    @Override
    public void execute(){
        super.execute();
        Log.i("Kicker Down Command ", "Running");
        kicker.moveKickerDown();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        Log.i("Kicker Down Command: ", "Is Command going to run: " + String.valueOf((Math.abs(robotHardware.getSpindexPositionFromEncoder() - LAUNCH_POS_1) < KICKING_TOLERANCE)));
        return (Math.abs(robotHardware.getSpindexPositionFromEncoder() - LAUNCH_POS_1) < KICKING_TOLERANCE);
//        return true; // only run the command once for now, add later
    }

}
