package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runSubsystem1Motor checks if the motor is not running and runs the intake  <BR>
 *     stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */
public class MajorArm {

    public Servo majorClawServo;

    public enum MAJOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }

    public DcMotorEx majorArmMotor;

    public enum MAJOR_ARM_STATE {
        PICKUP,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        CAPSTONE,
        PARKED,
    }

    /**
     * Parameter that register all hardware devices for MajorArm Subsystem
     * @param hardwareMap
     */
    public MajorArm(HardwareMap hardwareMap) {
        majorArmMotor = hardwareMap.get(DcMotorEx.class, "major_arm_motor");
        majorClawServo = hardwareMap.servo.get("major_claw_servo");
        initMajorArm();
    }

    public static int baselineEncoderCount = 0;
    public static final double CLAW_OPEN = 0.55;
    public static final double CLAW_CLOSED = 0.85;
    public MAJOR_CLAW_STATE majorClawState = MAJOR_CLAW_STATE.OPEN;
    public static int PICKUP_POSITION_COUNT = -700;
    public static int LEVEL1_POSITION_COUNT = -650;
    public static int LEVEL2_POSITION_COUNT = -600;
    public static int LEVEL3_POSITION_COUNT = -550;
    public static int CAPSTONE_POSITION_COUNT = -450;
    public static int PARKED_POSITION_COUNT = 0;
    public static int MAJORARM_DELTA_COUNT = 25;
    public int currentArmPositionCount = PARKED_POSITION_COUNT;
    public MAJOR_ARM_STATE currentMajorArmState = MAJOR_ARM_STATE.PARKED;
    public MAJOR_ARM_STATE previousMajorArmState = MAJOR_ARM_STATE.PARKED;

    public static double ARM_MOTOR_POWER = 0.2;
    public static double ARM_MOTOR_DELTA_POWER = 0.1;

    public boolean runArmToLevelState = false;

    /**
     * When init is pressed, the arm will reset and set the claw and armMotor values to default
     */
    public void initMajorArm(){
        resetArm();
        turnArmBrakeModeOff();
        majorArmMotor.setPositionPIDFCoefficients(5.0);
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.OPEN;
        majorArmMotor.setTargetPosition(PARKED_POSITION_COUNT);
        majorArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        currentMajorArmState = MAJOR_ARM_STATE.PARKED;
        previousMajorArmState = MAJOR_ARM_STATE.PARKED;
    }

    /**
     * sets MajorArm to a specific position and power depending on the level wanted
     * @param power
     */
    public void runMajorArmToLevel(double power){
        majorArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true) {//|| majorArmMotor.isBusy() == true){
            majorArmMotor.setPower(power);
            runArmToLevelState = false;
        } else {
            majorArmMotor.setPower(0.0);
        }
    }

    /**
     * Stops and resets the encoder values of majorArm
     */
    public void resetArm(){
        DcMotor.RunMode runMode = majorArmMotor.getMode();
        majorArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        majorArmMotor.setMode(runMode);
    }

    /**
     * Turns the armBrakeMode On
     */
    public void turnArmBrakeModeOn(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Turns the armBrakeMode Off
     */
    public void turnArmBrakeModeOff(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     *Changes the claw state to Open if it is initally closed and vice versa
     */
    public void changeMajorClawState() {
        if ((majorClawState == MAJOR_CLAW_STATE.OPEN)) {
            closeMajorClaw();
        } else if ((majorClawState == MAJOR_CLAW_STATE.CLOSED)) {
            openMajorClaw();
        }
    }

    /**
     * Closes the claw servo and sets state to CLOSED
     */
    public void closeMajorClaw(){
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
    }

    /**
     * Opens the claw servo and sets state to OPEN
     */
    public void openMajorClaw(){
        majorClawServo.setPosition(CLAW_OPEN);
        majorClawState = MAJOR_CLAW_STATE.OPEN;
    }

    /**
     * Changes the level of the Arm to Capstone
     */
    public void moveMajorArmCapstonePosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(CAPSTONE_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.CAPSTONE;
    }

    /**
     * Changes the level of the Arm to Pickup
     */
    public void moveMajorArmPickupPosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(PICKUP_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.PICKUP;
    }

    /**
     * Changes the level of the Arm to Level One
     */
    public void moveMajorArmLevel1Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL1_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_1;
    }

    /**
     * Changes the level of the Arm to Level 2
     */
    public void moveMajorArmLevel2Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL2_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_2;
    }

    /**
     * Changes the level of the Arm to Level three
     */
    public void moveMajorArmLevel3Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL3_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_3;
    }

    /**
     * Changes the level of the Arm to Parking
     */
    public void moveMajorArmParkingPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(PARKED_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.PARKED;
    }

    /**
     * Move Major Arm Slightly Down
     */
    public void moveMajorArmSlightlyDown(){
        if (//(currentArmPositionCount >= PARKED_POSITION_COUNT) &&
                currentArmPositionCount >= PICKUP_POSITION_COUNT + MAJORARM_DELTA_COUNT){
            turnArmBrakeModeOn();
            currentArmPositionCount = currentArmPositionCount - MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(currentArmPositionCount);
            runArmToLevelState = true;
        }
    }

    /**
     * MoveMajor Arm Slightly Up
     */
    public void moveMajorArmSlightlyUp(){
        if ((//currentArmPositionCount > PICKUP_POSITION_COUNT) &&
                currentArmPositionCount <= PARKED_POSITION_COUNT - MAJORARM_DELTA_COUNT)){
            turnArmBrakeModeOn();
            currentArmPositionCount = currentArmPositionCount + MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(currentArmPositionCount);
            runArmToLevelState = true;
        }
    }


    /**
     * Changes the level of the Arm up One
     */
    public void moveMajorArmUpOne() {
        if ((currentMajorArmState == MAJOR_ARM_STATE.PICKUP)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel1Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_1)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel2Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_2)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel3Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_3)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmCapstonePosition();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.CAPSTONE)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmParkingPosition();
            return;
        }
    }

    /**
     * Changes the level of the Arm down One
     */
    public void moveMajorArmDownOne() {
        if ((currentMajorArmState == MAJOR_ARM_STATE.PARKED)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmCapstonePosition();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.CAPSTONE)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel3Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_3)){
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel2Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_2)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmLevel1Position();
            return;
        }
        if ((currentMajorArmState == MAJOR_ARM_STATE.LEVEL_1)) {
            previousMajorArmState = currentMajorArmState;
            moveMajorArmPickupPosition();
            return;
        }
    }

    public int getMajorArmPositionCount(){
        return majorArmMotor.getCurrentPosition();
    }

    public MAJOR_CLAW_STATE getMajorClawState() {
        return majorClawState;
    }

    public MAJOR_ARM_STATE getMajorArmPosition() {
        return currentMajorArmState;
    }

    //TODO: Add code MajorArm Slight drop (reduce by 50 counts, dont change state when left trigger is pressed.

}