package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */
public class Hand {
    //Initialization of <hand servo's>
    public Servo wristServo;
    public Servo gripServo;

    //Initialization of HAND_STATE and HAND_GRIP_STATE and HAND_MOTOR_POSITION enums
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN,
        CLOSE
    }
    public GRIP_STATE gripState = GRIP_STATE.CLOSE;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        WRIST_UP_MAX,
        WRIST_UP,
        WRIST_LEVEL,
        WRIST_DOWN
    }
    public WRIST_STATE wristState = WRIST_STATE.WRIST_UP_MAX;

    //constants for Hand and grip position
    public static final double OPEN_GRIP_POS = 0.45; //value of Grip to open
    public static final double CLOSE_GRIP_POS = 0; //value of Grip to close
    public static final double CLOSE_GRIP_FULL_POSITION = 0.00;

    public static final double WRIST_DOWN_POSITION = 0.5;
    public static final double WRIST_DOWN_MIN_POSITION = 0.43;
    public static final double WRIST_PICKUP_LEVEL_POSITION = 0.60;
    public static final double WRIST_LOW_LEVEL_POSITION = 0.56;
    public static final double WRIST_DEFAULT_LEVEL_POSITION = 0.58;
    public static final double WRIST_MEDIUM_LEVEL_POSITION = 0.51;
    public static final double WRIST_HIGH_LEVEL_POSITION = 0.48;
    public static final double WRIST_DEFAULT_UP_POSITION = 0.66;
    public static final double WRIST_UP_MAX_POSITION = 0.70;

    public double wristLevelPosition = WRIST_DEFAULT_LEVEL_POSITION;
    public double wristUpPosition = WRIST_PICKUP_LEVEL_POSITION;

    public Hand(HardwareMap hardwareMap) { //map hand servo's to each
        gripServo = hardwareMap.get(Servo.class, "gripServo");
        wristServo = hardwareMap.get(Servo.class, "wristServo");
        initHand();
    }

    //initialize arm
    public void initHand() {
        moveWristUpMax();
        closeGrip();
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        gripServo.setPosition(OPEN_GRIP_POS);
        gripState = GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        gripServo.setPosition(CLOSE_GRIP_POS);
        gripState = GRIP_STATE.CLOSE;
    }

    public void toggleGrip(){
        if (gripState == GRIP_STATE.CLOSE) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    //rotates wrist to level position
    public void moveWristLevel(double shoulderLevelPosition){
        determineWristLevelPosition(shoulderLevelPosition);
        wristServo.setPosition(wristLevelPosition);
        wristState = WRIST_STATE.WRIST_LEVEL;
    }

    //rotates hand up given controller input
    public void moveWristUpMax(){
        wristServo.setPosition(WRIST_UP_MAX_POSITION);
        wristState = WRIST_STATE.WRIST_UP_MAX;
    }

    //rotates hand up given controller input
    public void moveWristUp(double shoulderLevelPosition){
        determineWristLevelPosition(shoulderLevelPosition);
        wristServo.setPosition(wristUpPosition);
        wristState = WRIST_STATE.WRIST_UP;
    }

    //rotates hand down given controller input
    public void moveWristDown(){
        wristServo.setPosition(WRIST_DOWN_POSITION);
        wristState = WRIST_STATE.WRIST_DOWN;
    }

    public void determineWristLevelPosition(double shoulderPosition){
        //wristLevelPos = SystemState.ShoulderAngleRadians - radianCount; //TODO: Test Logic
        switch (SystemState.ShoulderState) {
            case PICKUP:
            case GROUND_JUNCTION:
                wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION;
                break;
            case LOW_JUNCTION:
                wristLevelPosition = WRIST_LOW_LEVEL_POSITION;
                break;
            case RANDOM:
            case DYNAMIC_PICKUP_MINIMUM:
            case MAX_RAISED:
                wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION + ((shoulderPosition - SystemState.SHOULDER_PICKUP_POSITION)
                        / SystemState.SHOULDER_WRIST_ANGLE_FACTOR);
                break;
            case MEDIUM_JUNCTION:
                wristLevelPosition = WRIST_MEDIUM_LEVEL_POSITION;
                break;
            case HIGH_JUNCTION:
                wristLevelPosition = WRIST_HIGH_LEVEL_POSITION;
                break;
        }
        wristUpPosition = wristLevelPosition + 0.06;
    }
}
