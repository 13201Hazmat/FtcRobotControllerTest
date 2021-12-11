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

    public static final double CLAW_OPEN = 0.45;
    public static final double CLAW_CLOSED = 0.80;
    public MAJOR_CLAW_STATE majorClawState = MAJOR_CLAW_STATE.OPEN;

    public static int majorarmMotorBaselineEncoderCount = 0;
    public static int MAJORARM_MOTOR_PICKUP_POSITION_COUNT = -750;
    public static int MAJORARM_MOTOR_LEVEL1_POSITION_COUNT = -660;
    public static int MAJORARM_MOTOR_LEVEL2_POSITION_COUNT = -575;
    public static int MAJORARM_MOTOR_LEVEL3_POSITION_COUNT = -500;
    public static int MAJORARM_MOTOR_CAPSTONE_POSITION_COUNT = -425;
    public static int MAJORARM_MOTOR_PARKED_POSITION_COUNT = 0;
    public static int MAJORARM_DELTA_COUNT = 25;
    public int majorarmCurrentArmPositionCount = MAJORARM_MOTOR_PARKED_POSITION_COUNT;
    public MAJOR_ARM_STATE currentMajorArmState = MAJOR_ARM_STATE.PARKED;
    public MAJOR_ARM_STATE previousMajorArmState = MAJOR_ARM_STATE.PARKED;

    public static final double MAJORARM_WRIST_PICKUP_POSITION = 0.3;
    public static final double  MAJORARM_WRIST_LEVEL1_POSITION = 0.45;
    public static final double  MAJORARM_WRIST_LEVEL2_POSITION = 0.60;
    public static final double  MAJORARM_WRIST_LEVEL3_POSITION = 0.75;
    public static final double  MAJORARM_WRIST_CAPSTONE_POSITION = 0.90;
    public static final double  MAJORARM_WRIST_PARKED_POSITION = 1.0;
    public static final double  MAJORARM_WRIST_INIT_POSITION = 0.1;

    public static double MAJORARM_MOTOR_POWER = 0.2;
    public static double MAJORARM_MOTOR_DELTA_POWER = 0.1;

    public boolean runMajorArmToLevelState = false;
    public boolean moveWrist = false;

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

    public void runMajorArmToLevel(double power){
        majorArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runMajorArmToLevelState == true) {//|| majorArmMotor.isBusy() == true){
            majorArmMotor.setPower(power);
            runMajorArmToLevelState = false;
        } else {
            majorArmMotor.setPower(0.0);
        }
    }
    public void resetArm(){
        DcMotor.RunMode runMode = majorArmMotor.getMode();
        majorArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        majorArmMotor.setMode(runMode);
    }

    public void turnArmBrakeModeOn(){
        majorArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

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

    public void changeMajorClawState() {
        if ((majorClawState == MAJOR_CLAW_STATE.OPEN)) {
            closeMajorClaw();
        } else if ((majorClawState == MAJOR_CLAW_STATE.CLOSED)) {
            openMajorClaw();
        }
    }

    public void closeMajorClaw(){
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
    }

    public void openMajorClaw(){
        if (currentMajorArmState != MAJOR_ARM_STATE.PARKED && currentMajorArmState != MAJOR_ARM_STATE.INIT) {
            majorClawServo.setPosition(CLAW_OPEN);
            majorClawState = MAJOR_CLAW_STATE.OPEN;
        }
    }

    //change the level of the Arm to Capstone
    public void moveMajorArmCapstonePosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_CAPSTONE_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.CAPSTONE;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_CAPSTONE_POSITION);
    }

    //change the level of the Arm to Pickup
    public void moveMajorArmPickupPosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_PICKUP_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.PICKUP;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_PICKUP_POSITION);
    }

    //change the level of the arm to Level One
    public void moveMajorArmLevel1Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL1_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_1;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_LEVEL1_POSITION);
    }

    //change the level of the arm to Level Two
    public void moveMajorArmLevel2Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL2_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_2;
        moveWristWithTimer();
        //majorWristServo.setPosition(MAJORARM_WRIST_LEVEL2_POSITION);
    }

    //change the level of the arm to level three
    public void moveMajorArmLevel3Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(MAJORARM_MOTOR_LEVEL3_POSITION_COUNT + majorarmMotorBaselineEncoderCount);
        runMajorArmToLevelState = true;
        currentMajorArmState = MAJOR_ARM_STATE.LEVEL_3;
        moveWristWithTimer();
    }

    //change the level of the arm to the parking
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


    //change the level of the Arm to Level 1
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

    //change the level of the Arm by one Down
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