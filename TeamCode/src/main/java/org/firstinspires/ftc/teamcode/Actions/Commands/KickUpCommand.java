package org.firstinspires.ftc.teamcode.Actions.Commands;

import static org.firstinspires.ftc.teamcode.Actions.Commands.moveToEmptySpindexSlot.LAUNCH_POS_1;
import static org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem.KICKING_TOLERANCE;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.SpindexSubsystemReal;
import org.firstinspires.ftc.teamcode.subsystems.KickerSubsystem;
@Config
public class KickUpCommand extends CommandBase {
    public static int kickerTimerBuffer = 500;
    private ElapsedTime kickerTimer = new ElapsedTime();
    private KickerSubsystem kicker;
    private RobotHardware robotHardware;
    public KickUpCommand(KickerSubsystem kicker, RobotHardware robotHardware){
        this.robotHardware = robotHardware;
        this.kicker = kicker;
    }

    @Override
    public void initialize() {
        super.initialize();
        kickerTimer.reset();
        Log.i("Kicker Up Command", "Initialized");
    }

    @Override
    public void execute(){
        super.execute();
        Log.i("Kicker Up Command: ", String.valueOf(kickerTimer.milliseconds()));
        Log.i("Kicker Up Command ", "Running");
        kicker.moveKickerUp();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        Log.i("Kicker Up Command: ", "Is Command going to run, by spindex pos: " + String.valueOf((Math.abs(robotHardware.getSpindexPositionFromEncoder() - LAUNCH_POS_1) < KICKING_TOLERANCE)));
//        Log.i("Kicker Up Command: ", "Is command going to run by timer: " + String.valueOf(kickerTimer.milliseconds() > kickerTimerBuffer));
        return (Math.abs(robotHardware.getSpindexPositionFromEncoder() - LAUNCH_POS_1) < KICKING_TOLERANCE);
//        return true; // only run the command once for now, add later
    }

}
