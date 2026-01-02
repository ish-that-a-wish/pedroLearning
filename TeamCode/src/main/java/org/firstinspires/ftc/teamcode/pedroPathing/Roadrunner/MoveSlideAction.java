package org.firstinspires.ftc.teamcode.pedroPathing.Roadrunner;

import android.util.Log;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class MoveSlideAction implements Action {
    private int Ticks;
    private DcMotorEx slideMotor;

    private boolean initialized = false;
    private ElapsedTime timer;
    public MoveSlideAction(HardwareMap hardwareMap, int ticks, boolean dir){
        Log.i("MOVING SLIDE, ", "TICKS: " + ticks);
        slideMotor = hardwareMap.get(DcMotorEx.class, "slideMotor");
        if(!dir){
            Log.i("CHANGING SLIDE DIRECTION", "FORWARD");
            slideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else{
            Log.i("CHANGING SLIDE DIRECTION ", "REVERSE");
            slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        slideMotor.setTargetPosition(0);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        Ticks = ticks;
    }
    @Override
    public boolean run(@NonNull TelemetryPacket telemetryPacket) {
        if (!initialized) {
            timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
            initialized = true;
        }
        Log.i("MOVING SLIDE", "TICKS: " + Ticks);
        slideMotor.setTargetPosition(Ticks);
        slideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        slideMotor.setPower(0.5);
        return false;
    }
}
