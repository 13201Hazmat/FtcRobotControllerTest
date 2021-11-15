package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class MinorArm {

    public Servo minorClawServo;

    public enum MINOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }

    public Servo minorArmServo;

    public enum MINOR_SERVO_STATE {
        PICKUP,
        LEVEL_1,
        PARKED,
    }

    public MinorArm(HardwareMap hardwareMap) {
        minorArmServo = hardwareMap.servo.get("minor_arm_servo");
        minorClawServo = hardwareMap.servo.get("minor_claw_servo");
    }

    public static final double CLAW_OPEN = 0.8;
    public static final double CLAW_CLOSED = 0.55;
    public MINOR_CLAW_STATE minorClawState = MINOR_CLAW_STATE.OPEN;
    public static final double PICKUP_POSITION_COUNT = 0.85;
    public static final double LEVEL1_POSITION_COUNT = 0.72;
    public static final double PARKED_POSITION_COUNT = 0.45;
    public MINOR_SERVO_STATE minorServoState = MINOR_SERVO_STATE.PARKED;
    public MINOR_SERVO_STATE previousMinorServoState = MINOR_SERVO_STATE.PARKED;


    public void initMinorArm(){
        minorClawServo.setPosition(CLAW_CLOSED);
        minorClawState = MINOR_CLAW_STATE.CLOSED;
        minorArmServo.setPosition(PARKED_POSITION_COUNT);
        minorServoState = MINOR_SERVO_STATE.PARKED;
        previousMinorServoState = MINOR_SERVO_STATE.PARKED;
    }

    public MINOR_CLAW_STATE getMinorClawState() {
        return minorClawState;
    }

    public MINOR_SERVO_STATE getMinorServoState() {
        return minorServoState;
    }

    public void changeMinorClawState() {
        if ((minorClawState == MINOR_CLAW_STATE.OPEN)) {
            minorClawServo.setPosition(CLAW_CLOSED);
            minorClawState = MINOR_CLAW_STATE.CLOSED;
        } else if ((minorClawState == MINOR_CLAW_STATE.CLOSED)) {
            minorClawServo.setPosition(CLAW_OPEN);
            minorClawState = MINOR_CLAW_STATE.OPEN;
        }
    }

    //change the level of the Arm to Pickup
    public void moveMinorArmPickupPosition() {
        minorArmServo.setPosition(PICKUP_POSITION_COUNT);
        minorServoState = MINOR_SERVO_STATE.PICKUP;
    }

    //change the level of the arm to Level One
    public void moveMinorArmLevel1Position() {
        minorArmServo.setPosition(LEVEL1_POSITION_COUNT);
        minorServoState = MINOR_SERVO_STATE.PICKUP;
    }

    //change the level of the arm to the parking
    public void moveMinorArmParkingPosition() {
        minorArmServo.setPosition(PARKED_POSITION_COUNT);
        minorServoState = MINOR_SERVO_STATE.PICKUP;
    }

    //change the level of the Arm to Level 1
    public void moveMinorArmUpOne() {
        if ((minorServoState == MINOR_SERVO_STATE.PICKUP)) {
            previousMinorServoState = minorServoState;
            moveMinorArmLevel1Position();
            return;
        }
        if ((minorServoState == MINOR_SERVO_STATE.LEVEL_1)) {
            previousMinorServoState = minorServoState;
            moveMinorArmParkingPosition();
            return;
        }
    }

    //change the level of the Arm by one Down
    public void moveMinorArmDownOne() {
        if ((minorServoState == MINOR_SERVO_STATE.PARKED)) {
            previousMinorServoState = minorServoState;
            moveMinorArmLevel1Position();
            return;
        }

        if ((minorServoState == MINOR_SERVO_STATE.PARKED)) {
            previousMinorServoState = minorServoState;
            moveMinorArmPickupPosition();
            return;
        }
    }
}