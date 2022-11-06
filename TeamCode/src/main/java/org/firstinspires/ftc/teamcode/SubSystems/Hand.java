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
    public Servo intakeLeftServo;
    public Servo intakeRightServo;


    //Initialization of HAND_STATE and HAND_GRIP_STATE and HAND_MOTOR_POSITION enums
    public HAND_GRIP_STATE handGripState;
    public WRIST_STATE wristState;

    //constants for Hand and grip position
    int openGripPos = 1; //value of Grip to open
    int closeGripPos = 0; //value of Grip to close

    public boolean runHandToLevelState = false;
    public static final double WRIST_UP_POSITION = 180;//get position from robot
    public static final double WRIST_DOWN_POSITION = -180;//get position from robot
    public static final double WRIST_DEFAULT_LEVEL_POSITION = 0;//get position from robot
    public double wristLevelPosition = WRIST_DEFAULT_LEVEL_POSITION;

    //Hand - wrist, grip enum declaration
    public enum WRIST_STATE {
        WRIST_UP,
        WRIST_LEVEL,
        WRIST_DOWN
    }
    public enum HAND_GRIP_STATE{ //state of the Hand Grip
        OPEN,
        CLOSE
    }




    public Hand(HardwareMap hardwareMap) { //map hand servo's to each
        //gripServo = hardwareMap.get(Servo.class, "gripServo");
        //intakeLeftServo = hardwareMap.get(Servo.class, "intakeLeftServo");
        //intakeRightServo = hardwareMap.get(Servo.class, "intakeRightServo");
        initHand();
    }

    //initialize arm
    public void initHand(){
        wristServo.setPosition((int) WRIST_DEFAULT_LEVEL_POSITION);
    }
    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        if (handGripState != HAND_GRIP_STATE.OPEN){

            gripServo.setPosition(openGripPos);
            intakeRightServo.setPosition(openGripPos);
            intakeLeftServo.setPosition(openGripPos);
            handGripState = HAND_GRIP_STATE.OPEN;

        }
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        if (handGripState != HAND_GRIP_STATE.CLOSE) {
            gripServo.setPosition(closeGripPos);
            intakeLeftServo.setPosition(closeGripPos);
            intakeRightServo.setPosition(closeGripPos);
            handGripState = HAND_GRIP_STATE.CLOSE;


        }
    }

    //rotates wrist to level position
    public void moveWristLevel(){
        wristLevelPosition = determineWristLevelPosition();
        wristServo.setPosition(wristLevelPosition);
        wristState = WRIST_STATE.WRIST_LEVEL;
    }
    //rotates hand up given controller input
    public void moveWristUp(){
        wristServo.setPosition((int) WRIST_UP_POSITION);
        wristState = WRIST_STATE.WRIST_UP;
    }

    //rotates hand down given controller input
    public void moveWristDown(){
        wristServo.setPosition((int) WRIST_DOWN_POSITION);
        wristState = WRIST_STATE.WRIST_DOWN;
    }

    public double determineWristLevelPosition(){
        //TODO: FILLED based on Shoulder angle
        return 0;
    }
}
