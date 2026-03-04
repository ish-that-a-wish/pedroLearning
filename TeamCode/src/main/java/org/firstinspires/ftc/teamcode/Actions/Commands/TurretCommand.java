package org.firstinspires.ftc.teamcode.Actions.Commands;

import android.util.Log;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
@Config
public class TurretCommand extends CommandBase {
    private final double turretTicksPerRev = 751.8 * 5; // for torque increase
    private Pose RED_GOAL_POSE = new Pose(132, 132, Math.toRadians(45)); // pose of the red goal
    private Follower follower;
    private Pose pose;
    private DcMotorEx turret;
    private HardwareMap hardwareMap;
    public static double power = 0.2;
    public TurretCommand(Follower follower, HardwareMap hardwareMap){
        this.hardwareMap = hardwareMap;
        this.follower = follower;
    }
    @Override
    public void initialize() {
        super.initialize();
        turret = hardwareMap.get(DcMotorEx.class, "LaunchTurretMotor");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turret.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turret.setPower(power);
        Log.i("INTAKE", "Initialized");
    }

    @Override
    public void execute(){
        pose = follower.getPose(); // get the current pose
        Log.i("Turret: ","Amount to move to goal: " +  calculateAmountToTurnToGoal());
        Log.i("Turret: ", "Positional alignment pos: " + pose);
        Log.i("Turret: ", "Degrees to ticks: " + degreesToTicks(calculateAmountToTurnToGoal()));
        super.execute();
       updateTurret();
    }

    @Override
    public boolean isFinished(){
        super.isFinished();
        return false; // always update turret
    }
    public double calculateAmountToTurnToGoal(){
        double amountToTurn = Math.atan2(RED_GOAL_POSE.getY() - pose.getY(), RED_GOAL_POSE.getX() - pose.getX());
        return Math.toDegrees(amountToTurn - pose.getHeading());
    }
    public double degreesToTicks(double deg){
        return (deg/360) * turretTicksPerRev;
    }
    public void updateTurret(){
        double deg = calculateAmountToTurnToGoal();
        int ticks = (int) degreesToTicks(deg);
        turret.setDirection(ticks < 0 ? DcMotorEx.Direction.FORWARD : DcMotorEx.Direction.REVERSE);
        Log.i("Turret: ", "Current Direction: " + turret.getDirection());
        turret.setTargetPosition(Math.abs(ticks));
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setVelocity(2000);
    }
}
