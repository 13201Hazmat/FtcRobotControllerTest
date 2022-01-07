package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake.
 * Major Arm consists of a Motor and a Servo. The motor allows the arm to raise in height, and the servo adds a "grip" functionality.
 *
 * The states are as followed: <BR>
 *     <emsp>MAJOR_CLAW_STATE = if the Major Claw is Open or Closed </emsp> <BR>
 *     <emsp>ARM_STATE = if the Major Arm is at Pickup, Level 1, Level 2, Level 3, Capstone, or Parking Level </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>initMajorArm initializes the Major Arm, and gets it ready to be utilized </emsp> <BR>
 *     <emsp>runArmToLevel moves the Major Arm to one of the values indicated by ARM_STATE </emsp> <BR>
 *     <emsp>turnArmBrakeModeOn turns Brake Mode On for the Major Arm</emsp> <BR>
 *     <emsp>turnArmBrakeModeOff turns Brake MOde Off for the Major Arm</emsp> <BR>
 *     <emsp>changeClawState changes the MAJOR_CLAW_STATE the Major Arm from open to closed, or vice versa</emsp> <BR>
 *     <emsp>closeClaw closes the claw of the Major Arm </emsp> <BR>
 *     <emsp>openClaw opens the claw of the Major Arm</emsp> <BR>
 *     <emsp>moveMajorArmCapstonePosition moves the Major Arm to the value indicated by ARM_STATE for CAPSTONE</emsp> <BR>
 *     <emsp>moveMajorArmParkingPosition moves the Major Arm to the value indicated by ARM_STATE for PARKED</emsp> <BR>
 *     <emsp>moveMajorArmPickupPosition moves the Major Arm to the value indicated by ARM_STATE for PICKUP</emsp> <BR>
 *     <emsp>moveMajorArmLevel1Position moves the Major Arm to the value indicated by ARM_STATE for LEVEL_1</emsp> <BR>
 *     <emsp>moveMajorArmLevel2Position moves the Major Arm to the value indicated by ARM_STATE for LEVEL_2</emsp> <BR>
 *     <emsp>moveMajorArmLevel3Position moves the Major Arm to the value indicated by ARM_STATE for LEVEL_3</emsp> <BR>
 *     <emsp>moveMajorArmSlightlyDown moves the Major Arm slightly down from its current position </emsp> <BR>
 *     <emsp>moveMajorArmSlightlyUp moves the Major Arm slightly up from its current position </emsp> <BR>
 *     <emsp>moveArmUpOne moves the Major Arm up by one position based on ARM_STATE</emsp> <BR>
 *     <emsp>moveArmDownOne moves the Major Arm down by one position based on ARM_STATE </emsp> <BR>
 *     <emsp>moveArmDownOne </emsp> <BR>
 *     <emsp>getArmPosition returns the ARM_STATE that the Major Arm is currently at</emsp> <BR>
 *     <emsp>getMajorArmPositionCount returns the precise numeric value of the Major Arm current position</emsp> <BR>
 *     <emsp>getMajorClawState returns the MAJOR_CLAW_STATE; if the claw of the Major Arm is opened or closed</emsp> <BR>
 */
public class MajorArm {

    public Servo majorClawServo;
    public Servo majorWristServo;

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
        INIT
    }

    public MajorArm(HardwareMap hardwareMap) {
        majorArmMotor = hardwareMap.get(DcMotorEx.class, "major_arm_motor");
        majorWristServo = hardwareMap.servo.get("major_wrist_servo");
        majorClawServo = hardwareMap.servo.get("major_claw_servo");
        initMajorArm();
    }

    public static final double CLAW_OPEN = 0.35;
    public static final double CLAW_CLOSED = 0.0;
    public MAJOR_CLAW_STATE majorClawState = MAJOR_CLAW_STATE.OPEN;

    public static int majorarmMotorBaselineEncoderCount = 0;
    public static int MAJORARM_MOTOR_PICKUP_POSITION_COUNT = -775;//-750;
    public static int MAJORARM_MOTOR_LEVEL1_POSITION_COUNT = -660;
    public static int MAJORARM_MOTOR_LEVEL2_POSITION_COUNT = -575;
    public static int MAJORARM_MOTOR_LEVEL3_POSITION_COUNT = -500;
    public static int MAJORARM_MOTOR_CAPSTONE_POSITION_COUNT = -365;
    public static int MAJORARM_MOTOR_PARKED_POSITION_COUNT = 0;
    public static int MAJORARM_DELTA_COUNT = 25;
    public int majorarmCurrentArmPositionCount = MAJORARM_MOTOR_PARKED_POSITION_COUNT;
    public MAJOR_ARM_STATE currentMajorArmState = MAJOR_ARM_STATE.PARKED;
    public MAJOR_ARM_STATE previousMajorArmState = MAJOR_ARM_STATE.PARKED;

    public static final double MAJORARM_WRIST_PICKUP_POSITION = 0.55;
    public static final double  MAJORARM_WRIST_LEVEL1_POSITION = 0.45;
    public static final double  MAJORARM_WRIST_LEVEL2_POSITION = 0.60;
    public static final double  MAJORARM_WRIST_LEVEL3_POSITION = 0.75;
    public static final double  MAJORARM_WRIST_CAPSTONE_POSITION = 0.90;
    public static final double  MAJORARM_WRIST_PARKED_POSITION = 0.98;
    public static final double  MAJORARM_WRIST_INIT_POSITION = 0.22;

    public static double MAJORARM_MOTOR_POWER = 0.2;
    public static double MAJORARM_MOTOR_DELTA_POWER = 0.1;

    public boolean runMajorArmToLevelState = false;
    public boolean moveWrist = false;

    /**
     * initializes the Major Arm, and gets it ready to be utilized
     */
    public void initMajorArm(){
        resetArm();
        turnArmBrakeModeOff();
        majorArmMotor.setPositionPIDFCoefficients(5.0);
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_PARKED_POSITION_COUNT);
        majorArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        currentMajorArmState = MAJOR_ARM_STATE.PARKED;
        previousMajorArmState = MAJOR_ARM_STATE.PARKED;
    }

    /**
     * moves the Major Arm to one of the values indicated by ARM_STATE
     */
    public void runMajorArmToLevel(double power){
        majorArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runMajorArmToLevelState == true) {//|| majorArmMotor.isBusy() == true){
            majorArmMotor.setPower(power);
            runMajorArmToLevelState = false;
        } else {
            majorArmMotor.setPower(0.0);
        }
    }

    /**
     * resets the Major Arm
     */
    public void resetArm(){
        DcMotor.RunMode runMode = majorArmMotor.getMode();
        majorArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        majorArmMotor.setMode(runMode);
    }

    /**
     * turns Brake Mode On for the Major Arm
     */
    public void turnArmBrakeModeOn(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * turns Brake MOde Off for the Major Arm
     */
    public void turnArmBrakeModeOff(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }


    ElapsedTime timer = new ElapsedTime(MILLISECONDS);
    public void moveWristWithTimer(){
        timer.reset();
        moveWrist = true;
    }

    public boolean delayWristTimerReached(double time){
        if (timer.time() < time){
            return false;
        } else  {
            return true;
        }
    }

    public void moveMajorArmWristToInitPosition(){
        majorWristServo.setPosition(MAJORARM_WRIST_INIT_POSITION);
    }

    public void moveMajorArmWristToParkedPosition(){
        majorWristServo.setPosition(MAJORARM_WRIST_PARKED_POSITION);
    }

    public void moveMajorArmWristToPickupPosition(){
        majorWristServo.setPosition(MAJORARM_WRIST_PICKUP_POSITION);
    }

    public void moveMajorArmWristToPosition() {
        double delayTime;
        if (currentMajorArmState == MAJOR_ARM_STATE.PICKUP) {
            delayTime = 0;
        } else {
            delayTime = 500;
        }
        if (delayWristTimerReached(delayTime) && moveWrist) {
            switch (currentMajorArmState) {
                case PICKUP:
                    majorWristServo.setPosition(MAJORARM_WRIST_PICKUP_POSITION);
                    break;
                case LEVEL_1:
                    majorWristServo.setPosition(MAJORARM_WRIST_LEVEL1_POSITION);
                    break;
                case LEVEL_2:
                    majorWristServo.setPosition(MAJORARM_WRIST_LEVEL2_POSITION);
                    break;
                case LEVEL_3:
                    majorWristServo.setPosition(MAJORARM_WRIST_LEVEL3_POSITION);
                    break;
                case CAPSTONE:
                    majorWristServo.setPosition(MAJORARM_WRIST_CAPSTONE_POSITION);
                    break;
                case PARKED:
                    majorWristServo.setPosition(MAJORARM_WRIST_PARKED_POSITION);
                    break;
                case INIT:
                    majorWristServo.setPosition(MAJORARM_WRIST_INIT_POSITION);
                    break;
            }
            moveWrist = false;
        }
        return;
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void changeMajorClawState() {
        if ((majorClawState == MAJOR_CLAW_STATE.OPEN)) {
            closeMajorClaw();
        } else if ((majorClawState == MAJOR_CLAW_STATE.CLOSED)) {
            openMajorClaw();
        }
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void closeMajorClaw(){
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void openMajorClaw(){
        if (currentMajorArmState != MAJOR_ARM_STATE.PARKED && currentMajorArmState != MAJOR_ARM_STATE.INIT) {
            majorClawServo.setPosition(CLAW_OPEN);
            majorClawState = MAJOR_CLAW_STATE.OPEN;
        }
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmCapstonePosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_CAPSTONE_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.CAPSTONE;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_CAPSTONE_POSITION);
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmPickupPosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_PICKUP_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.PICKUP;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_PICKUP_POSITION);
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmLevel1Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL1_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_1;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_LEVEL1_POSITION);
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmLevel2Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL2_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_2;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_LEVEL2_POSITION);
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmLevel3Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL3_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_3;
        moveWristWithTimer();
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public void moveMajorArmParkingPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_PARKED_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.PARKED;
        moveWristWithTimer();
        closeMajorClaw();
    }

    /**
     * Move Major Arm Slightly Down
     */
    public void moveMajorArmSlightlyDown(){
        if (//(currentArmPositionCount >= PARKED_POSITION_COUNT) &&
                majorarmCurrentArmPositionCount >= MAJORARM_MOTOR_PICKUP_POSITION_COUNT + MAJORARM_DELTA_COUNT){
            turnArmBrakeModeOn();
            majorarmCurrentArmPositionCount = majorarmCurrentArmPositionCount - MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(majorarmCurrentArmPositionCount);
            runMajorArmToLevelState = true;
        }
    }

    /**
     * MoveMajor Arm Slightly Up
     */
    public void moveMajorArmSlightlyUp(){
        if ((//currentArmPositionCount > PICKUP_POSITION_COUNT) &&
                majorarmCurrentArmPositionCount <= MAJORARM_MOTOR_PARKED_POSITION_COUNT - MAJORARM_DELTA_COUNT)){
            turnArmBrakeModeOn();
            majorarmCurrentArmPositionCount = majorarmCurrentArmPositionCount + MAJORARM_DELTA_COUNT;
            majorArmMotor.setTargetPosition(majorarmCurrentArmPositionCount);
            runMajorArmToLevelState = true;
        }
    }


    /**
     * Method that takes keypad inputs to select the Autonomous options
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
     * Method that takes keypad inputs to select the Autonomous options
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

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public int getMajorArmPositionCount(){
        return majorArmMotor.getCurrentPosition();
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public MAJOR_CLAW_STATE getMajorClawState() {
        return majorClawState;
    }

    /**
     * Method that takes keypad inputs to select the Autonomous options
     */
    public MAJOR_ARM_STATE getMajorArmPosition() {
        return currentMajorArmState;
    }

    //TODO: Add code MajorArm Slight drop (reduce by 50 counts, dont change state when left trigger is pressed.

}