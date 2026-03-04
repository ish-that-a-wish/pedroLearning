package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;

import org.firstinspires.ftc.teamcode.common.RobotHardware;

//EXAMPLE CLASS FOR NOW WITHOUT PROPER INDEXING
public class SpindexSubsystem extends SubsystemBase {
    public static double DELTA_BETWEEN_POSITIONS = 0.375;
    public static double INTAKE_POS_1 = 0.195;
    public static double INTAKE_POS_2 = INTAKE_POS_1 + DELTA_BETWEEN_POSITIONS; //0.57;
    public static double INTAKE_POS_3 = INTAKE_POS_2 + DELTA_BETWEEN_POSITIONS; //0.945;

    public static double LAUNCH_POS_1 = INTAKE_POS_1 + (DELTA_BETWEEN_POSITIONS * 1.5); //0.7575
    public static double LAUNCH_POS_3 = LAUNCH_POS_1 - DELTA_BETWEEN_POSITIONS; // 0.3825;
    public static double LAUNCH_POS_2 = LAUNCH_POS_3 - DELTA_BETWEEN_POSITIONS; //0.0075;
    public boolean initMove = false; // have a init move var so only able to init once
    public enum SpindexPoses {
        INTAKE_POSE_1,
        INTAKE_POSE_2,
        INTAKE_POSE_3,
        LAUNCH_POSE_1,

        LAUNCH_POSE_2,
        LAUNCH_POSE_3
    }
    private SpindexPoses currentPose;
    private RobotHardware robotHardware;
    public SpindexSubsystem(RobotHardware robotHardware){
        this.robotHardware = robotHardware;
    }
    public boolean spindexFree(){
        return robotHardware.getSpindexPositionFromEncoder() > robotHardware.getSpindexPosition() - 0.05 && robotHardware.getSpindexPositionFromEncoder() < robotHardware.getSpindexPosition() + 0.05;
    }
    public void moveToPose(SpindexPoses pose){
        switch (pose){
            case INTAKE_POSE_1:
                robotHardware.setSpindexPosition(INTAKE_POS_1);
                currentPose = SpindexPoses.INTAKE_POSE_1;
                break;
            case INTAKE_POSE_2:
                robotHardware.setSpindexPosition(INTAKE_POS_2);
                currentPose = SpindexPoses.INTAKE_POSE_2;
                break;
            case INTAKE_POSE_3:
                robotHardware.setSpindexPosition(INTAKE_POS_3);
                currentPose = SpindexPoses.INTAKE_POSE_3;
                break;
            case LAUNCH_POSE_1:
                robotHardware.setSpindexPosition(LAUNCH_POS_1);
                currentPose = SpindexPoses.LAUNCH_POSE_1;
                break;
            case LAUNCH_POSE_2:
                robotHardware.setSpindexPosition(LAUNCH_POS_2);
                currentPose = SpindexPoses.LAUNCH_POSE_2;
                break;
            case LAUNCH_POSE_3:
                robotHardware.setSpindexPosition(LAUNCH_POS_3);
                currentPose = SpindexPoses.LAUNCH_POSE_3;
                break;
        }
    }
    public SpindexPoses getCurrentPose(){
        return currentPose;
    }
    public void initMove(){
        if(!initMove){
            robotHardware.setSpindexPosition(INTAKE_POS_1);
            initMove = true;
            currentPose = SpindexPoses.INTAKE_POSE_1;
        }
    }
    public boolean isReadyToLaunch(){
        return currentPose == SpindexPoses.LAUNCH_POSE_1; // return true if the current pose is launch pose
    }
    public void intakeBalls(){
        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_1) moveToPose(SpindexPoses.INTAKE_POSE_2);

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_2) moveToPose(SpindexPoses.INTAKE_POSE_3);

        if(robotHardware.didBallDetectionBeamBreak() && spindexFree() && currentPose == SpindexPoses.INTAKE_POSE_3) moveToPose(SpindexPoses.LAUNCH_POSE_1);
    }
    public double convertSpindexPoseToDouble(SpindexPoses pose){
        switch (pose){
            case INTAKE_POSE_1:
                return INTAKE_POS_1;
            case INTAKE_POSE_2:
                return INTAKE_POS_2;
            case INTAKE_POSE_3:
                return INTAKE_POS_3;
            case LAUNCH_POSE_1:
                return LAUNCH_POS_1;
            case LAUNCH_POSE_2:
                return LAUNCH_POS_2;
            case LAUNCH_POSE_3:
                return LAUNCH_POS_3;
        }
        return robotHardware.getSpindexPositionFromEncoder(); // defualt to the current spindex pose
    }
}