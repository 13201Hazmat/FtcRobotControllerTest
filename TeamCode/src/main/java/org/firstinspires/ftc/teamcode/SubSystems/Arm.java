package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DigitalChannel;

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
    public DigitalChannel armTouchSensor;  // Hardware Device Object

    //Arm states
    public enum ARM_MOTOR_STATE {
        PICKUP,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        MAX_EXTENDED,
        RANDOM
    }
    public ARM_MOTOR_STATE armMotorState = ARM_MOTOR_STATE.PICKUP;

    //Constants for Arm Standard positions
    public static final int PICKUP_WHILE_FACING_FORWARD_POSITION = 0;
    public static final int GROUND_JUNCTION_WHILE_FACING_FORWARD_POSITION = 0;
    public static final int LOW_JUNCTION_POSITION = (int) 0;
    public static final int MEDIUM_JUNCTION_POSITION = (int) 450;
    public static final int HIGH_JUNCTION_POSITION = (int) 1749;
    public static final int MAX_EXTENDED_POSITION_4_SLIDES = (int) 4500;
    public static final int MAX_EXTENDED_POSITION_3_SLIDES = (int) 3375;
    public static final int MAX_EXTENDED_POSITION_2_SLIDES = (int) 2250;
    public static final int MAX_EXTENDED_POSITION = MAX_EXTENDED_POSITION_2_SLIDES;

    public int armCurrentPosition = PICKUP_WHILE_FACING_FORWARD_POSITION; //Default arm position count
    public int armNewPosition = PICKUP_WHILE_FACING_FORWARD_POSITION;

    public static final double CONE_2_POSITION = 1000;
    public static final double CONE_3_POSITION = 1200;
    public static final double CONE_4_POSITION = 1300;
    public static final double CONE_5_POSITION = 1400;
    public static final double ENCODER_TO_LENGTH = 1/574 * 2; //TODO - fix this value to millimeter

    public int pickupArmWhileDynamicTurretPosition = 0;

    public static final int ARM_DELTA_COUNT_MAX = 200; //need tested values

    //Different constants of arm speed
    public static final double ARM_POWER = 0.5;

    public int armDeltaCount = 0; //Need tested value
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
        resetArm();
        turnArmBrakeModeOff();
        armMotorState = ARM_MOTOR_STATE.PICKUP;
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setPositionPIDFCoefficients(5.0);
        armMotor.setTargetPosition(PICKUP_WHILE_FACING_FORWARD_POSITION);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE);
        moveArmToPickUpWhileTurretFacingForward();
    }

    //Turns on the brake for arm motor
    public void turnArmBrakeModeOn(){
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm motor off
    public void turnArmBrakeModeOff() {
        armMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets arm position to ground junction
    public void moveArmToPickUpWhileTurretFacingForward(){
        //TODO : This should be used only when the Turret is facing forward (to avoid arm hitting the sides of the robot)
        turnArmBrakeModeOn();
        if (SystemState.TurretState == Turret.TURRET_MOTOR_STATE.FACING_FORWARD) {
            armMotor.setTargetPosition(PICKUP_WHILE_FACING_FORWARD_POSITION);
            runArmToLevelState = true;
        }
    }

    //TODO: Set arm position when below Low junction angle dynamically to avoid hitting side of the robot
    public void moveArmToPickUpWhileDynamicTurretAngle(double turretAngle){
        pickupArmWhileDynamicTurretPosition = 0; // TODO: Update with formula
        armMotor.setTargetPosition(pickupArmWhileDynamicTurretPosition);
        runArmToLevelState = true;
    }

    //Sets arm position to low junction
    public void moveArmToLowJunction(){
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(LOW_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    //Sets arm position or mid junction
    public void moveArmToMidJunction(){
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(MEDIUM_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    //Sets arm position to high junction
    public void moveArmToHighJunction(){
        turnArmBrakeModeOn();
        armMotor.setTargetPosition(HIGH_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    public void modifyArmLength(double stepSizeFactor){
        armDeltaCount = (int) stepSizeFactor * ARM_DELTA_COUNT_MAX;
        if (armDeltaCount !=0) {
            armCurrentPosition = armMotor.getCurrentPosition();
            armNewPosition = (int) (armCurrentPosition + armDeltaCount);
            if (armNewPosition < PICKUP_WHILE_FACING_FORWARD_POSITION
                    /*&& SystemState.TurretState == Turret.TURRET_MOTOR_STATE.FACING_FORWARD TODO*/) {
                armNewPosition = PICKUP_WHILE_FACING_FORWARD_POSITION;
                armMotorState = ARM_MOTOR_STATE.PICKUP;
            } else if (armNewPosition > MAX_EXTENDED_POSITION) {
                armNewPosition = MAX_EXTENDED_POSITION;
                armMotorState = ARM_MOTOR_STATE.MAX_EXTENDED;
            } else {
                armMotorState = ARM_MOTOR_STATE.RANDOM;
            }
            if (armNewPosition != armCurrentPosition) {
                turnArmBrakeModeOn();
                armMotor.setTargetPosition(armNewPosition);
                runArmToLevelState = true;
            }
        }
    }

    public double calculateMaxExtensionArmEncoderPositionBasedOnShoulderAngle(){
        double maxExtensionArmEncoderPositionBasedOnShoulderAngle = 0; //TODO: Measure arm max extension in mm

        //TODO: if shoulderAngle > Threshold when arm at full extension will touch the ground
        maxExtensionArmEncoderPositionBasedOnShoulderAngle = MAX_EXTENDED_POSITION;

        return maxExtensionArmEncoderPositionBasedOnShoulderAngle;
    }

    public void convertMotorEncoderValueToArmLength(){
        int convertedMotorEncoderValueToArmLength = (int) (armMotor.getCurrentPosition() * ENCODER_TO_LENGTH); //TODO: From encoder value Find Max length and write proportional convertion algorithm
        SystemState.ArmExtension = convertedMotorEncoderValueToArmLength;
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
    }

    //Resets the arm
    public void resetArm(){
        DcMotorEx.RunMode runMode = armMotor.getMode();
        armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(runMode);

    }

    public void manualResetArm(double joystickValue){
        //uses the limit switch to reset position
        if (armTouchSensor.getState() == true) {
            turnArmBrakeModeOn();
            armMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        } else {
            armMotor.setTargetPosition((int) (armMotor.getCurrentPosition() - joystickValue * 50));
            runArmToLevel(0.1); //TODO: need tested value
        }
    }

    //declaring cone pickup positions, original array for setting array to default
    public double[] pickupPos = {
            PICKUP_WHILE_FACING_FORWARD_POSITION,
            CONE_2_POSITION,
            CONE_3_POSITION,
            CONE_4_POSITION,
            CONE_5_POSITION
    };

    //function for picking cones from stack
    public void pickupCone(double position){
        armMotor.setTargetPosition((int) position);
        runArmToLevel(AutonomousArmPower);
        runArmToLevelState = true;
    }
}





