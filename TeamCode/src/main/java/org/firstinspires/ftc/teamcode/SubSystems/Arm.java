package org.firstinspires.ftc.teamcode.SubSystems;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Definition of Arm Class <BR>
 *
 * Arm consists of linear slides starting from the shoulder(pivot) and can extend to a fixed length
 * by the driver, holding the hand at the end<BR>
 *
 * The states are as followed: <BR>
 *     ARM_STATE - example if linear slides are either fully extended or fully retracted  <BR>
 *     ARM_MOTOR_POSITION -  linear slides are at preset positions <BR>
 *
 * The functions are as followed: <BR>
 *     initArm resets the motors and positions <BR>
 *     turnArmBrakeModeOn and turnArmBrakeModeOff puts the arm motor in a stopped or active state <BR>
 *     moveToJunction functions set the target position to preset positions corresponding to function<BR>
 *     extendArm and retractArm functions extend and retract arm based on a delta value determined <BR>
 *     by the joystick <BR>
 *     resetArm resets the arm to the original position and states <BR>
 *     runArmToLevel runs the arm to the levels determined by the other functions <BR>
 */

public class Arm {
    //Initialization of armmotor
    public DcMotorEx armMotor;

    //Arm Motor : 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    public static final double ARM_MOTOR_ENCODER_TICKS = 537.7;

    public DigitalChannel armTouchSensor;  // Hardware Device Object

    //Arm states
    public enum ARM_STATE {
        MIN_RETRACTED,
        PICKUP,
        GROUND_JUNCTION,
        PICKUP_WRIST_DOWN_POSITION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        DYNAMIC_MAXIMUM,
        MAX_EXTENDED,
        RANDOM
    }
    public ARM_STATE armState = ARM_STATE.PICKUP;

    //Constants for Arm Standard positions
    public static final double MIN_RETRACTED_POSITION = 0;
    public static final double PICKUP_POSITION = 550;
    public static final double GROUND_JUNCTION_POSITION = PICKUP_POSITION;
    public static final double PICKUP_WRIST_DOWN_POSITION = 100;
    public static final double LOW_JUNCTION_POSITION = (int) 0;
    public static final double MEDIUM_JUNCTION_POSITION = (int) 450;
    public static final double HIGH_JUNCTION_POSITION = (int) 1555;
    public static final double MAX_EXTENDED_POSITION_2_SLIDES = (int) 2250;
    public static final double MAX_EXTENDED_POSITION_3_SLIDES = (int) 3250;//3375;
    public static final double MAX_EXTENDED_POSITION_4_SLIDES = (int) 4500;
    public static final double MAX_EXTENDED_POSITION = MAX_EXTENDED_POSITION_3_SLIDES;
    public double dynamicMaxExtendedPosition = MAX_EXTENDED_POSITION_3_SLIDES;

    public double armCurrentPosition = PICKUP_POSITION; //Default arm position count
    public double armCurrentMM = 0;
    public double armNewPosition = PICKUP_POSITION;
    public double armMMNewPosition = 0;

    public static final double CONE_2_POSITION = 1000;
    public static final double CONE_3_POSITION = 1200;
    public static final double CONE_4_POSITION = 1300;
    public static final double CONE_5_POSITION = 1400;
    public static final double ARM_ENCODER_TO_MM = 1/574 * 2; //TODO - fix this value to millimeter
    public static final double ARM_MINIMUM_MM = 390; //Min in mm for arm

    public static final double ARM_DELTA_COUNT_MAX = 200;//200;//200 //need tested values
    public static final double ARM_DELTA_COUNT_RESET = 50;

    //Different constants of arm speed
    public static final double ARM_POWER_EXTEND = 0.8;
    public static final double ARM_POWER_RETRACT = 0.95;
    public static final double ARM_AUTO_POWER_EXTEND = 0.9;
    public static final double ARM_AUTO_POWER_RETRACT = 0.95;
    public enum ARM_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }
    public ARM_MOVEMENT_DIRECTION armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;

    public double armDeltaCount = 0; //Need tested value
    public static double AutonomousArmPower = 2;//need tested value

    public boolean runArmToLevelState = false;

    //Constructor`
    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armMotor");

        // get a reference to our digitalTouch object.
        armTouchSensor = hardwareMap.get(DigitalChannel.class, "armTouch");

        // set the digital channel to input.
        armTouchSensor.setMode(DigitalChannel.Mode.INPUT);
        initArm();
    }

    //Method is able to initialize the arm
    public void initArm(){
        resetArmMode();
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setPositionPIDFCoefficients(5.0);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turnArmBrakeModeOn();
        manualResetArm();
    }

    //Turns on the brake for arm motor
    public void turnArmBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm motor off
    /*public void turnArmBrakeModeOff() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }*/

    public void moveArmToMinRetracted(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < MIN_RETRACTED_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)MIN_RETRACTED_POSITION);
        armState = ARM_STATE.MIN_RETRACTED;
        runArmToLevelState = true;
    }

    //Sets arm position to ground junction
    public void moveArmToPickUp(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < PICKUP_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)PICKUP_POSITION);
        armState = ARM_STATE.PICKUP;
        runArmToLevelState = true;
    }

    //Sets arm position to ground junction
    public void moveArmToPickUpWristDown(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < PICKUP_WRIST_DOWN_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)PICKUP_WRIST_DOWN_POSITION);
        armState = ARM_STATE.PICKUP_WRIST_DOWN_POSITION;
        runArmToLevelState = true;
    }

    //Sets arm position to low junction
    public void moveArmToLowJunction(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < LOW_JUNCTION_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)LOW_JUNCTION_POSITION);
        armState = ARM_STATE.LOW_JUNCTION;
        runArmToLevelState = true;
    }

    //Sets arm position or mid junction
    public void moveArmToMediumJunction(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < MEDIUM_JUNCTION_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)MEDIUM_JUNCTION_POSITION);
        armState = ARM_STATE.MEDIUM_JUNCTION;
        runArmToLevelState = true;
    }

    //Sets arm position to high junction
    public void moveArmToHighJunction(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < HIGH_JUNCTION_POSITION ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int)HIGH_JUNCTION_POSITION);
        armState = ARM_STATE.HIGH_JUNCTION;
        runArmToLevelState = true;
    }

    public void moveArmToDynamicMaxExtended(){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < dynamicMaxExtendedPosition ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int) dynamicMaxExtendedPosition);
        armState = ARM_STATE.DYNAMIC_MAXIMUM;
        runArmToLevelState = true;
    }

    public void modifyArmLength(double stepSizeFactor){
        armDeltaCount = stepSizeFactor * ARM_DELTA_COUNT_MAX;
        if (armDeltaCount !=0) {
            armCurrentPosition = armMotor.getCurrentPosition();
            armNewPosition = (armCurrentPosition + armDeltaCount);
            if (armNewPosition < MIN_RETRACTED_POSITION) {
                armNewPosition = MIN_RETRACTED_POSITION;
                armState = ARM_STATE.MIN_RETRACTED;
            } else if (armNewPosition > MAX_EXTENDED_POSITION) {
                armNewPosition = MAX_EXTENDED_POSITION;
                armState = ARM_STATE.MAX_EXTENDED;
            } else {
                armState = ARM_STATE.RANDOM;
            }
            armCurrentPosition = armMotor.getCurrentPosition();
            if (armCurrentPosition < armNewPosition ) {
                armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
            } else {
                armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
            }
            if (armNewPosition != armCurrentPosition) {
                turnArmBrakeModeOn();
                armMotor.setTargetPosition((int)armNewPosition);
                runArmToLevelState = true;
            }
        }
    }

    public void moveArmToLength(double armLength){
        turnArmBrakeModeOn();
        armCurrentPosition = armMotor.getCurrentPosition();
        if (armCurrentPosition < dynamicMaxExtendedPosition ) {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.EXTEND;
        } else {
            armMovementDirection = ARM_MOVEMENT_DIRECTION.RETRACT;
        }
        armMotor.setTargetPosition((int) armLength);
        armState = ARM_STATE.RANDOM;
        runArmToLevelState = true;
    }

    public void moveArmExtensionBasedOnShoulderAngle(double shoulderCurrentPosRadians, double shoudlerNewPosRadians){
        armCurrentMM = convertMotorEncoderValueToArmMM(armCurrentPosition);
        armMMNewPosition = ARM_MINIMUM_MM * (1 - (armCurrentMM *
                (Math.cos(shoudlerNewPosRadians - Math.PI/4)/
                        Math.cos(shoulderCurrentPosRadians - Math.PI/4))));
        armNewPosition = convertArmMMToEncoderValue(armMMNewPosition);
        if(armNewPosition < MIN_RETRACTED_POSITION){
            armNewPosition = MIN_RETRACTED_POSITION;
        }
        if(armNewPosition> MAX_EXTENDED_POSITION){
            armNewPosition = MAX_EXTENDED_POSITION;
        }
        if(armNewPosition != armCurrentPosition){
            moveArmToLength(armNewPosition);
        }
    }

    public double convertMotorEncoderValueToArmMM(double armMotorPosition){
        double convertedMotorEncoderValueToArmMM = (armMotorPosition * ARM_ENCODER_TO_MM); //TODO: From encoder value Find Max length and write proportional convertion algorithm
        return convertedMotorEncoderValueToArmMM;
    }

    public double convertArmMMToEncoderValue(double armMMPosition){
        double convertedArmMMToEncoderValue = armMMPosition / ARM_ENCODER_TO_MM;
        return convertedArmMMToEncoderValue;
    }

    //sets the arm motor power
    public void runArmToLevel(double power){
        armMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true){
            armMotor.setPower(power);
            runArmToLevelState = false;
        } else{
            armMotor.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO */
        if (!armTouchSensor.getState()) {
            resetArmMode();
        }
    }

    //Resets the arm
    public void resetArmMode(){
        DcMotorEx.RunMode runMode = armMotor.getMode();
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(runMode);
    }

    public void manualResetArm(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (armTouchSensor.getState() && timer.time() < 5000) {
            armMotor.setTargetPosition((int) (armMotor.getCurrentPosition() - ARM_DELTA_COUNT_RESET));
            runArmToLevelState = true;
            runArmToLevel(ARM_POWER_RETRACT);
        }
        resetArmMode();
        turnArmBrakeModeOn();
        armState = ARM_STATE.MIN_RETRACTED;
    }

}





