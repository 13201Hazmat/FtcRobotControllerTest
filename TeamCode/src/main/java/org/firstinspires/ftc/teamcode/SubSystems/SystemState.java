package org.firstinspires.ftc.teamcode.SubSystems;

import org.firstinspires.ftc.teamcode.AadiGeometry.AadiVector;

/**
 * Static Class to capture States of all subsystems and pass to each other
 */

public class SystemState {
    public static Arm.ARM_STATE ArmState = Arm.ARM_STATE.PICKUP;
    public static Hand.GRIP_STATE HandGripState = Hand.GRIP_STATE.OPEN;
    public static Hand.WRIST_STATE HandWristState = Hand.WRIST_STATE.WRIST_LEVEL;
    public static Shoulder.SHOULDER_STATE ShoulderState = Shoulder.SHOULDER_STATE.PICKUP;
    public static Turret.TURRET_MOTOR_STATE TurretState = Turret.TURRET_MOTOR_STATE.FACING_FORWARD;

    public static double ShoulderAngleRadians = 0;
    public static double TurretAngleRadians = 0;

    public static double ArmExtension = 0; //Length of the arm in centimeters
    public static double ARM_EXTENSION_POWER_MULTIPLIER = 0.05;

    public static final double ARM_LENGTH_TO_MM_RATIO = 1;
    public static final double SHOULDER_ANGLE_TO_RADIANS_RATIO = Shoulder.SHOULDER_MOTOR_ENCODER_TICKS/Math.PI ;
    public static final double TURRET_ANGLE_TO_RADIANS_RATIO = Turret.TURRET_MOTOR_ENCODER_TICKS/Math.PI;

    public static final double SHOULDER_PICKUP_POSITION = Shoulder.PICKUP_POSITION;
    public static final double SHOULDER_WRIST_ANGLE_FACTOR = (Shoulder.HIGH_JUNCTION_POSITION - Shoulder.PICKUP_POSITION)/
            (Hand.WRIST_HIGH_LEVEL_POSITION - Hand.WRIST_PICKUP_LEVEL_POSITION);

    // To protect Arm from hitting the ground
    public static final double SHOULDER_ARM_FACTOR = ((Shoulder.THRESHOLD_POSITION - Shoulder.PICKUP_POSITION)/
            ((Arm.MAX_EXTENDED_POSITION - Arm.PICKUP_POSITION)));

    public static final AadiVector MIN_RETRACTED_AADI_VECTOR
            = new AadiVector(Arm.MIN_RETRACTED_POSITION, Shoulder.PICKUP_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector PICKUP_AADI_VECTOR
            = new AadiVector(Arm.PICKUP_POSITION, Shoulder.PICKUP_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector GROUND_JUNCTION_AADI_VECTOR
            = new AadiVector(Arm.GROUND_JUNCTION_POSITION, Shoulder.GROUND_JUNCTION_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector PICKUP_WRIST_DOWN_AADI_VECTOR
            = new AadiVector(Arm.PICKUP_POSITION, Shoulder.PICKUP_WRIST_DOWN_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector LOW_JUNCTION_AADI_VECTOR
            = new AadiVector(Arm.LOW_JUNCTION_POSITION,Shoulder.LOW_JUNCTION_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector MEDIUM_JUNCTION_AADI_VECTOR
            = new AadiVector(Arm.MEDIUM_JUNCTION_POSITION,Shoulder.MEDIUM_JUNCTION_POSITION, AadiVector.WRIST_ANGLE.UP);

    public static final AadiVector HIGH_JUNCTION_AADI_VECTOR
            = new AadiVector(Arm.HIGH_JUNCTION_POSITION, Shoulder.HIGH_JUNCTION_POSITION, AadiVector.WRIST_ANGLE.UP);

}
