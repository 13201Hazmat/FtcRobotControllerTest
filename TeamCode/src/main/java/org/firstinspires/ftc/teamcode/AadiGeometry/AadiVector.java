package org.firstinspires.ftc.teamcode.AadiGeometry;

import org.firstinspires.ftc.teamcode.SubSystems.SystemState;

public class AadiVector {
    private double armLength; // Arm Length in Encoder value
    private double shoulderAngle; //Shoulder Angle in Encoder value
    public enum WRIST_ANGLE {
        UP,
        LEVEL
    }
    private WRIST_ANGLE wristAngle = WRIST_ANGLE.UP;

    public AadiVector(double armLength, double shoulderAngle, WRIST_ANGLE wristAngle){
        this.armLength = armLength;
        this.shoulderAngle = shoulderAngle;
        this.wristAngle = wristAngle;
    }

    public void setArmLength(double armLength) {
        this.armLength = armLength;
    }

    public double getArmLength(AadiVector aadiVector){
        return aadiVector.armLength;
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

    public double getShoulderAngle(AadiVector aadiVector){
        return aadiVector.shoulderAngle;
    }

    public double shoulderAngleToRadians(double shoulderAngle){
        return shoulderAngle / SystemState.SHOULDER_ANGLE_TO_RADIANS_RATIO;
    }

    public static double shoulderRadiansToAngle(double shoulderRadians){
        return shoulderRadians * SystemState.SHOULDER_ANGLE_TO_RADIANS_RATIO;
    }

    public WRIST_ANGLE getWristAngle(AadiVector aadiVector){
        return aadiVector.wristAngle;
    }

    public void setWristAngle(WRIST_ANGLE wristAngle){
        this.wristAngle = wristAngle;
    }

}
