package org.firstinspires.ftc.teamcode.SubSystems;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Definition of Shoulder Class <BR>
 *
 * Shoulder raises or lowers the arm(linear slides) with 2 motors<BR>
 *
 * The states are as followed: <BR>
 *     SHOULDER_STATE - positions of the shoulder corresponding to scoring positions, or random(trigger control)  <BR>
 *
 * The functions are as followed: <BR>
 *     turnShoulderBrakeModeOn, turnShoulderBrakeModeOff - sets the shoulder motors to a state of brake or active<BR>
 *     initShoulder, resetShoulder - sets shoulder motor positions and sets states <BR>
 *     moveToShoulder[x] - moves the shoulder to [x] preset position <BR>
 *     raiseShoulder, lowerShoulder - raises and lowers the shoulder based on joystick control by changing delta value <BR>
 *     getShoulderPositionCount - returns the current right shoulder motor encoder count <BR>
 */

public class Shoulder {
    //Initialization of <Fill>
    public DcMotorEx rightShoulderMotor, leftShoulderMotor;

    //Initialization of <Fill>
    public enum SHOULDER_STATE {
        PICKUP, //PARKED, MIN_POSITION
        PICKUP_WRIST_DOWN,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        MAX_RAISED,
        RANDOM
    }
    public SHOULDER_STATE shoulderState = SHOULDER_STATE.PICKUP;

    public Turret turret;
    public DigitalChannel shoulderTouchSensor;  // Hardware Device Object

    public boolean runShoulderToLevelState = false;
    public boolean shoulderBelowThreshold = true;
    public double SHOULDER_DELTA_COUNT_MAX = 100;
    public double shoulderDeltaCount = 0; //Need tested value

    public static final double PICKUP_POSITION = 0;
    public static final double GROUND_JUNCTION_POSITION = 0; //Need tested values
    public static final double PICKUP_WRIST_DOWN_POSITION = 200;
    public static final double THRESHOLD_POSITION = 350;
    public static final double LOW_JUNCTION_POSITION = 380; //need tested values
    public static final double MEDIUM_JUNCTION_POSITION = 650; //need tested values
    public static final double HIGH_JUNCTION_POSITION = 808; //need tested values
    public static final double MAX_RAISED_POSITION = 900; //Need tested values

    public double dynamicMinPosition = PICKUP_POSITION;

    public double shoulderCurrentPosition = PICKUP_POSITION;
    public double shoulderNewPosition = PICKUP_POSITION; //Default shoulder position count

    public double shoulderAngleRadians, shoulderAngleDegrees;

    //Different constants of shoulder speed
    public static final double SHOULDER_POWER = 0.5;

    //Constructor
    public Shoulder(HardwareMap hardwareMap){
        leftShoulderMotor = hardwareMap.get(DcMotorEx.class, "lshMotor");
        rightShoulderMotor = hardwareMap.get(DcMotorEx.class, "rshMotor");
        // get a reference to our digitalTouch object.
        shoulderTouchSensor = hardwareMap.get(DigitalChannel.class, "shTouch");

        // set the digital channel to input.
        shoulderTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        initShoulder();
    }

    //Method is able to <Fill>
    public void initShoulder(){
        leftShoulderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShoulderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetShoulderMode();
        turnShoulderBrakeModeOff();
        leftShoulderMotor.setPositionPIDFCoefficients(5.0);
        rightShoulderMotor.setPositionPIDFCoefficients(5.0);
        leftShoulderMotor.setDirection(DcMotorEx.Direction.REVERSE);
        rightShoulderMotor.setDirection(DcMotorEx.Direction.FORWARD);
        moveShoulderToPickup();
    }

    public void turnShoulderBrakeModeOn() {
        leftShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnShoulderBrakeModeOff() {
        leftShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets shoulder position to ground junction
    public void moveShoulderToPickup() {
        turnShoulderBrakeModeOff();
        leftShoulderMotor.setTargetPosition((int)PICKUP_POSITION);
        rightShoulderMotor.setTargetPosition((int)PICKUP_POSITION);
        shoulderState = SHOULDER_STATE.PICKUP;
        runShoulderToLevelState = true;
    }

    //TODO: Set Shoulder position when below Low junction angle dynamically to avoid hitting side of the robot
    public void  moveShoulderToPickUpWristDown(){
        turnShoulderBrakeModeOn();
        leftShoulderMotor.setTargetPosition((int)PICKUP_WRIST_DOWN_POSITION);
        rightShoulderMotor.setTargetPosition((int)PICKUP_WRIST_DOWN_POSITION);
        shoulderState = SHOULDER_STATE.PICKUP_WRIST_DOWN;
        runShoulderToLevelState = true;
    }

    //Sets shoulder position to low junction
    public void moveToShoulderLowJunction() {
        turnShoulderBrakeModeOn();
        leftShoulderMotor.setTargetPosition((int)LOW_JUNCTION_POSITION);
        rightShoulderMotor.setTargetPosition((int)LOW_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.LOW_JUNCTION;
        runShoulderToLevelState = true;
    }

    //Sets shoulder position or mid junction
    public void moveToShoulderMediumJunction() {
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setTargetPosition((int)MEDIUM_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition((int)MEDIUM_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.MEDIUM_JUNCTION;
        runShoulderToLevelState = true;

    }

    //Sets shoulder position to high junction
    public void moveToShoulderHighJunction() {
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setTargetPosition((int)HIGH_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition((int)HIGH_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.HIGH_JUNCTION;
        runShoulderToLevelState = true;
    }

    public void moveShoulderToDynamicMinExtended(){
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setTargetPosition((int)dynamicMinPosition);
        leftShoulderMotor.setTargetPosition((int)dynamicMinPosition);
        shoulderState = SHOULDER_STATE.RANDOM;
        runShoulderToLevelState = true;
    }

    public void lowerShoulder(double stepSizeFactor) {
        shoulderDeltaCount = (int) stepSizeFactor * SHOULDER_DELTA_COUNT_MAX;
        if (shoulderDeltaCount !=0) {
            shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
            shoulderNewPosition = shoulderCurrentPosition - shoulderDeltaCount;
            if (shoulderNewPosition > PICKUP_POSITION) {
                shoulderState = SHOULDER_STATE.RANDOM;
                turnShoulderBrakeModeOn();
            } else {
                shoulderNewPosition = PICKUP_POSITION;
                shoulderState = SHOULDER_STATE.PICKUP;
                turnShoulderBrakeModeOff();
            }
            if (shoulderNewPosition != shoulderCurrentPosition) {
                rightShoulderMotor.setTargetPosition((int)shoulderNewPosition);
                leftShoulderMotor.setTargetPosition((int)shoulderNewPosition);
                runShoulderToLevelState = true;
            }
        }
    }

    public void raiseShoulder(double stepSizeFactor) {
        shoulderDeltaCount = (int) stepSizeFactor * SHOULDER_DELTA_COUNT_MAX;
        if (shoulderDeltaCount !=0) {
            shoulderCurrentPosition = (leftShoulderMotor.getCurrentPosition()
                    + rightShoulderMotor.getCurrentPosition())/2;
            shoulderNewPosition = shoulderCurrentPosition + shoulderDeltaCount;
            if (shoulderNewPosition < MAX_RAISED_POSITION) {
                shoulderState = SHOULDER_STATE.RANDOM;
            } else {
                shoulderNewPosition = MAX_RAISED_POSITION;
                shoulderState = SHOULDER_STATE.MAX_RAISED;
            }
            if (shoulderNewPosition != shoulderCurrentPosition) {
                turnShoulderBrakeModeOn();
                rightShoulderMotor.setTargetPosition((int)shoulderNewPosition);
                leftShoulderMotor.setTargetPosition((int)shoulderNewPosition);
                runShoulderToLevelState = true;
            }
        }
    }

    public void runShoulderToLevel(double power){
        rightShoulderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        leftShoulderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runShoulderToLevelState == true){
            leftShoulderMotor.setPower(power);
            rightShoulderMotor.setPower(power);
            runShoulderToLevelState = false;
        } else{
            leftShoulderMotor.setPower(0.0);
            rightShoulderMotor.setPower(0.0);
        }
    }

    //TODO : Calculate Shoulder Angle
    public void calculateShoulderAngle(){
        shoulderAngleRadians = rightShoulderMotor.getCurrentPosition() * Math.PI/537.7; //TODO : Calculate turret Angle
        shoulderAngleDegrees = Math.toDegrees(shoulderAngleRadians);
    }

    public void resetShoulderMode(){
        DcMotorEx.RunMode runMode = leftShoulderMotor.getMode();
        rightShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulderMotor.setMode(runMode);
        rightShoulderMotor.setMode(runMode);
    }

    public void manualResetShoulder(){
        //uses the limit switch to reset position
        /*if (shoulderTouchSensor.getState() == true) {
            turnShoulderBrakeModeOn();
            rightShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            leftShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shoulderState = SHOULDER_STATE.PICKUP;
        } else {
            rightShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - 50));
            leftShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - 50));
            runShoulderToLevel(0.2); //need tested value
        }*/
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while ((shoulderTouchSensor.getState() == true) && timer.time() < 3000) {
            rightShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - 50));
            leftShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - 50));
            runShoulderToLevel(0.2);
        }
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shoulderState = SHOULDER_STATE.PICKUP;
    }



}
