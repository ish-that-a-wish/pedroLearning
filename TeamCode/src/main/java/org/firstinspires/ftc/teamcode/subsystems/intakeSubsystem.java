package org.firstinspires.ftc.teamcode.subsystems;

import android.util.Log;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intakeSubsystem extends SubsystemBase {

    private HardwareMap hwmap;
    public Motor intake;

    public intakeSubsystem(HardwareMap hwmap){
        intake = new Motor(hwmap, "IntakeMotor", Motor.GoBILDA.RPM_1150);
    }

    public void runIntake(){
        intake.set(1);
    }

    public void stopIntake(){
        intake.set(0);
    }


    @Override
    public void periodic() {
        super.periodic();
        Log.i("INTAKE", "Called");
    }
}