package org.firstinspires.ftc.teamcode.AadiGeometry;

import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.SystemState;

public class AadiVector {
    private double armLength; // Arm Length in Encoder value
    private double shoulderAngle; //Shoulder Angle in Encoder value
    public Hand.WRIST_STATE wristState;

    public AadiVector(double armLength, double shoulderAngle, Hand.WRIST_STATE wristState){
        this.armLength = armLength;
        this.shoulderAngle = shoulderAngle;
        this.wristState = wristState;
    }

    public void setArmLength(double armLength) {
        this.armLength = armLength;
    }

    public double getArmLength(){
        return armLength;
    }

    public double armLengthToMM(double armLength){
        return armLength / SystemState.ARM_LENGTH_TO_MM_RATIO; //TODO : Create correct formula
    }

    public static double armMMtoLength(double armMM){
        return armMM * SystemState.ARM_LENGTH_TO_MM_RATIO; //TODO : Create correct formula
    }

    public void setShoulderAngle(double shoulderAngle) {
        this.shoulderAngle = shoulderAngle;
    }

    public double getShoulderAngle(){
        return shoulderAngle;
    }

    public double shoulderAngleToRadians(double shoulderAngle){
        return shoulderAngle / SystemState.SHOULDER_ANGLE_TO_RADIANS_RATIO;
    }

    public static double shoulderRadiansToAngle(double shoulderRadians){
        return shoulderRadians * SystemState.SHOULDER_ANGLE_TO_RADIANS_RATIO;
    }

    public Hand.WRIST_STATE getWristState(){
        return wristState;
    }

    public void setWristState(Hand.WRIST_STATE wristState){
        this.wristState = wristState;
    }

}
