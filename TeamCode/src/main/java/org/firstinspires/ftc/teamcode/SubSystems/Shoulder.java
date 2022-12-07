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

    //Shoulder Motor : 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Gearing : 1:5
    //public static final double SHOULDER_MOTOR_ENCODER_TICKS = 537.7 * 5;
    //Shoulder Motor : 5203 Series Yellow Jacket Planetary Gear Motor (26.9:1 Ratio, 24mm Length 8mm REX Shaft, 223 RPM, 3.3 - 5V Encoder)
    public static final double SHOULDER_MOTOR_ENCODER_TICKS = 751.8 * 5;


    //Initialization of <Fill>
    public enum SHOULDER_STATE {
        PICKUP, //PARKED, MIN_POSITION
        DYNAMIC_PICKUP_MINIMUM,
        GROUND_JUNCTION,
        PICKUP_WRIST_DOWN,
        DYNAMIC_PICKUP_WRIST_DOWN,
        LOW_JUNCTION,
        DYNAMIC_LOW_JUNCTION,
        MEDIUM_JUNCTION,
        DYNAMIC_MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        DYNAMIC_HIGH_JUNCTION,
        MAX_RAISED,
        RANDOM
    }
    public SHOULDER_STATE shoulderState = SHOULDER_STATE.PICKUP;

    public Turret turret;
    public DigitalChannel shoulderTouchSensor;  // Hardware Device Object

    public boolean runShoulderToLevelState = false;
    public boolean shoulderBelowThreshold = true;
    public double SHOULDER_DELTA_COUNT_MAX = 70; //50;
    public double SHOULDER_DELTA_COUNT_RESET = 70;
    public double shoulderDeltaCount = 0; //Need tested value

    public static final double PICKUP_POSITION = 0;
    public static final double PICKUP_POSITION_ARM_MAX_EXTENDED = 210; //150;
    public static final double GROUND_JUNCTION_POSITION = PICKUP_POSITION;
    public static final double PICKUP_WRIST_DOWN_POSITION = 115; //80;
    public static final double PICKUP_WRIST_DOWN_POSITION_ARM_MAX_EXTENDED = 140; //100;
    public static final double LOW_JUNCTION_POSITION = 325;//480; //336;
    public static final double LOW_JUNCTION_POSITION_ARM_MAX_EXTENDED = 470; //337;
    public static final double MEDIUM_JUNCTION_POSITION = 730;//864; //610;
    public static final double MEDIUM_JUNCTION_POSITION_ARM_MAX_EXTENDED = 615;//626;//440;
    public static final double HIGH_JUNCTION_POSITION = 1000;//950;//1060; //760;
    public static final double HIGH_JUNCTION_POSITION_ARM_MAX_EXTENDED = 760;//820; //591;
    public static final double MAX_RAISED_POSITION = 1100; //900;

    public double dynamicMinPosition = PICKUP_POSITION;

    public double shoulderCurrentPosition = PICKUP_POSITION;
    public double shoulderNewPosition = PICKUP_POSITION; //Default shoulder position count

    public double shoulderAngleRadians, shoulderAngleDegrees;

    //Different constants of shoulder speed
    public static final double SHOULDER_POWER_UP = 0.8;
    public static final double SHOULDER_POWER_DOWN = 0.8;

    public enum SHOULDER_MOVEMENT_DIRECTION {
        UP,
        DOWN
    }
    public SHOULDER_MOVEMENT_DIRECTION shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;

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
        manualResetShoulder();
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
        turnShoulderBrakeModeOff(); //TODO : See if Brake Mode needs to turn off for pickup
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = PICKUP_POSITION;
        leftShoulderMotor.setTargetPosition((int)PICKUP_POSITION);
        rightShoulderMotor.setTargetPosition((int)PICKUP_POSITION);
        shoulderState = SHOULDER_STATE.PICKUP;
        shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        runShoulderToLevelState = true;
    }

    //Set Shoulder position when below Low junction angle dynamically to avoid hitting side of the robot
    public void  moveShoulderToPickUpWristDown(){
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = PICKUP_WRIST_DOWN_POSITION;
        if (shoulderCurrentPosition < PICKUP_WRIST_DOWN_POSITION) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        leftShoulderMotor.setTargetPosition((int)PICKUP_WRIST_DOWN_POSITION);
        rightShoulderMotor.setTargetPosition((int)PICKUP_WRIST_DOWN_POSITION);
        shoulderState = SHOULDER_STATE.PICKUP_WRIST_DOWN;
        runShoulderToLevelState = true;
    }

    //Sets shoulder position to low junction
    public void moveToShoulderLowJunction() {
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = LOW_JUNCTION_POSITION;
        if (shoulderCurrentPosition < LOW_JUNCTION_POSITION) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        leftShoulderMotor.setTargetPosition((int)LOW_JUNCTION_POSITION);
        rightShoulderMotor.setTargetPosition((int)LOW_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.LOW_JUNCTION;
        runShoulderToLevelState = true;
    }

    //Sets shoulder position or mid junction
    public void moveToShoulderMediumJunction() {
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = MEDIUM_JUNCTION_POSITION;
        if (shoulderCurrentPosition < MEDIUM_JUNCTION_POSITION) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        rightShoulderMotor.setTargetPosition((int)MEDIUM_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition((int)MEDIUM_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.MEDIUM_JUNCTION;
        runShoulderToLevelState = true;

    }

    //Sets shoulder position to high junction
    public void moveToShoulderHighJunction() {
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = HIGH_JUNCTION_POSITION;
        if (shoulderCurrentPosition < HIGH_JUNCTION_POSITION) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        rightShoulderMotor.setTargetPosition((int)HIGH_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition((int)HIGH_JUNCTION_POSITION);
        shoulderState = SHOULDER_STATE.HIGH_JUNCTION;
        runShoulderToLevelState = true;
    }

    public void moveShoulderToDynamicMinExtended(){
        turnShoulderBrakeModeOff();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = dynamicMinPosition;
        if (shoulderCurrentPosition < dynamicMinPosition) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        rightShoulderMotor.setTargetPosition((int)dynamicMinPosition);
        leftShoulderMotor.setTargetPosition((int)dynamicMinPosition);
        shoulderState = SHOULDER_STATE.DYNAMIC_PICKUP_MINIMUM;
        runShoulderToLevelState = true;
    }

    public void moveToShoulderMaxRaised(){
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = MAX_RAISED_POSITION;
        if (shoulderCurrentPosition < MAX_RAISED_POSITION) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        rightShoulderMotor.setTargetPosition((int)MAX_RAISED_POSITION);
        leftShoulderMotor.setTargetPosition((int)MAX_RAISED_POSITION);
        shoulderState = SHOULDER_STATE.MAX_RAISED;
        runShoulderToLevelState = true;
    }

    public void moveShoulderToAngle(double shoulderAnglePosition){
        turnShoulderBrakeModeOn();
        shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
        shoulderNewPosition = shoulderAnglePosition;
        if (shoulderCurrentPosition < dynamicMinPosition) {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        } else {
            shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
        }
        rightShoulderMotor.setTargetPosition((int)shoulderAnglePosition);
        leftShoulderMotor.setTargetPosition((int)shoulderAnglePosition);
        shoulderState = SHOULDER_STATE.RANDOM;
        runShoulderToLevelState = true;
    }

    public void lowerShoulder(double stepSizeFactor) {
        shoulderDeltaCount = (int) stepSizeFactor * SHOULDER_DELTA_COUNT_MAX;
        shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.DOWN;
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
        shoulderMovementDirection = SHOULDER_MOVEMENT_DIRECTION.UP;
        if (shoulderDeltaCount !=0) {
            shoulderCurrentPosition = leftShoulderMotor.getCurrentPosition();
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
            shoulderCurrentPosition = shoulderNewPosition;
        } else{
            leftShoulderMotor.setPower(0.0);
            rightShoulderMotor.setPower(0.0);
        }
        /*if ((shoulderState == SHOULDER_STATE.PICKUP) && (shoulderTouchSensor.getState())) {
            manualResetShoulder();
        } Overheating motor? TODO*/
        if (!(shoulderTouchSensor.getState())){
            resetShoulderMode();
        }

    }

    public void calculateShoulderAngle(){
        shoulderAngleRadians = rightShoulderMotor.getCurrentPosition() * Math.PI/SHOULDER_MOTOR_ENCODER_TICKS;
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
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while ((shoulderTouchSensor.getState()) && timer.time() < 3000) {
            rightShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - SHOULDER_DELTA_COUNT_RESET));
            leftShoulderMotor.setTargetPosition((int) (rightShoulderMotor.getCurrentPosition() - SHOULDER_DELTA_COUNT_RESET));
            runShoulderToLevelState = true;
            runShoulderToLevel(0.2);
        }
        turnShoulderBrakeModeOn();
        resetShoulderMode();
        shoulderState = SHOULDER_STATE.PICKUP;
    }
}
