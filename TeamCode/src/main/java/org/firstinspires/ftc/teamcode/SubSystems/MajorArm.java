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

    public enum MAJOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }

    public DcMotorEx majorArmMotor;

    public enum ARM_STATE {
        PICKUP,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
        CAPSTONE,
        PARKED,
    }

    public MajorArm(HardwareMap hardwareMap) {
        majorArmMotor = hardwareMap.get(DcMotorEx.class, "major_arm_motor");
        majorClawServo = hardwareMap.servo.get("major_claw_servo");
    }

    public static int baselineEncoderCount = 0;
    public static final double CLAW_OPEN = 0.0;
    public static final double CLAW_CLOSED = 1.0;
    public MAJOR_CLAW_STATE majorClawState = MAJOR_CLAW_STATE.OPEN;
    public static int PICKUP_POSITION_COUNT = -750;
    public static int LEVEL1_POSITION_COUNT = -675;
    public static int LEVEL2_POSITION_COUNT = -625;
    public static int LEVEL3_POSITION_COUNT = -550;
    public static int CAPSTONE_POSITION_COUNT = -450;
    public static int PARKED_POSITION_COUNT = 0;
    public static int MAJORARM_DELTA_COUNT = 50;
    public int currentArmPositionCount = PARKED_POSITION_COUNT;
    public ARM_STATE currentArmState = ARM_STATE.PARKED;
    public ARM_STATE previousArmState = ARM_STATE.PARKED;

    public static double ARM_MOTOR_POWER = 0.8;

    public boolean runArmToLevelState = false;

    public void initMajorArm(){
        resetArm();
        turnArmBrakeModeOff();
        majorArmMotor.setPositionPIDFCoefficients(5.0);
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.OPEN;
        majorArmMotor.setTargetPosition(PARKED_POSITION_COUNT);
        majorArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        currentArmState = ARM_STATE.PARKED;
        previousArmState = ARM_STATE.PARKED;
    }

    public void runArmToLevel(double power){
        majorArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true) {//|| majorArmMotor.isBusy() == true){
            majorArmMotor.setPower(power);
            runArmToLevelState = false;
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

    public void changeClawState() {
        if ((majorClawState == MAJOR_CLAW_STATE.OPEN)) {
            closeClaw();
        } else if ((majorClawState == MAJOR_CLAW_STATE.CLOSED)) {
            openClaw();
        }
    }

    public void closeClaw(){
        majorClawServo.setPosition(CLAW_CLOSED);
        majorClawState = MAJOR_CLAW_STATE.CLOSED;
    }

    public void openClaw(){
        majorClawServo.setPosition(CLAW_OPEN);
        majorClawState = MAJOR_CLAW_STATE.OPEN;
    }

    //change the level of the Arm to Capstone
    public void moveArmCapstonePosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(CAPSTONE_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.CAPSTONE;
    }

    //change the level of the Arm to Pickup
    public void moveArmPickupPosition() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(PICKUP_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.PICKUP;
    }

    //change the level of the arm to Level One
    public void moveArmLevel1Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL1_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.LEVEL_1;
    }

    //change the level of the arm to Level Two
    public void moveArmLevel2Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL2_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.LEVEL_2;
    }

    //change the level of the arm to level three
    public void moveArmLevel3Position() {
        turnArmBrakeModeOn();
        majorArmMotor.setTargetPosition(LEVEL3_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.LEVEL_3;
    }

    //change the level of the arm to the parking
    public void moveArmParkingPosition() {
        turnArmBrakeModeOff();
        majorArmMotor.setTargetPosition(PARKED_POSITION_COUNT + baselineEncoderCount);
        runArmToLevelState = true;
        currentArmState = ARM_STATE.PARKED;
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


    //change the level of the Arm to Level 1
    public void moveArmUpOne() {
        if ((currentArmState == ARM_STATE.PICKUP)) {
            previousArmState = currentArmState;
            moveArmLevel1Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_1)) {
            previousArmState = currentArmState;
            moveArmLevel2Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_2)) {
            previousArmState = currentArmState;
            moveArmLevel3Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_3)) {
            previousArmState = currentArmState;
            moveArmCapstonePosition();
            return;
        }
        if ((currentArmState == ARM_STATE.CAPSTONE)) {
            previousArmState = currentArmState;
            moveArmParkingPosition();
            return;
        }
    }

    //change the level of the Arm by one Down
    public void moveArmDownOne() {
        if ((currentArmState == ARM_STATE.PARKED)) {
            previousArmState = currentArmState;
            moveArmCapstonePosition();
            return;
        }
        if ((currentArmState == ARM_STATE.CAPSTONE)) {
            previousArmState = currentArmState;
            moveArmLevel3Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_3)){
            previousArmState = currentArmState;
            moveArmLevel2Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_2)) {
            previousArmState = currentArmState;
            moveArmLevel1Position();
            return;
        }
        if ((currentArmState == ARM_STATE.LEVEL_1)) {
            previousArmState = currentArmState;
            moveArmPickupPosition();
            return;
        }
    }

    public int getMajorArmPositionCount(){
        return majorArmMotor.getCurrentPosition();
    }

    public MAJOR_CLAW_STATE getMajorClawState() {
        return majorClawState;
    }

    public ARM_STATE getArmPosition() {
        return currentArmState;
    }

    //TODO: Add code MajorArm Slight drop (reduce by 50 counts, dont change state when left trigger is pressed.

}