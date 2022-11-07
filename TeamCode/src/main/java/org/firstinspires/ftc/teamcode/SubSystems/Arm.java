package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
 *     extendArm and retractArm functions extend and retract arm based on a delta value determined
 *     by the joystick <BR>
 *     resetArm resets the arm to the original position and states <BR>
 *     runArmToLevel runs the arm to the levels determined by the other functions <BR>
 */

public class Arm {
    //Initialization of armmotor
    public DcMotorEx armmotor;

    //Arm states, either fully extended or retracted all the way

    //Initialization of ARM_MOTOR_POSITION
    public enum ARM_MOTOR_STATE {
        PICKUP, // PICKUP_
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        MAX_EXTENDED,
        RANDOM
    }

    //Initialization of ARM_MOTOR_POSITION and ARM_STATE enums
    public ARM_MOTOR_STATE armMotorState;

    //Constants for Arm positions
    public static final int PICKUP_WHILE_FACING_FORWARD_POSITION = 0; //Need tested values
    public static final int GROUND_JUNCTION_WHILE_FACING_FORWARD_POSITION = 0; //Need tested values
    public static final int LOW_JUNCTION_POSITION = (int) (537* 1.25)*3; //Need tested values
    public static final int MEDIUM_JUNCTION_POSITION = (int) (537* 2.5)*3; //Need tested values
    public static final int HIGH_JUNCTION_POSITION = (int) (537* 3.75)*3; //Need tested values
    public static final int MAX_EXTENDED_POSITION = (int) (537* 5)*3; //Need tested value

    public int pickupArmWhileDynamicTurretPosition = 0;

    public static final int AUTO_RETRACTION_DELTA_POSITION = 50; //need tested values
    public static final int ARM_DELTA_COUNT_MAX = 250; //need tested values

    //Different constants of arm speed
    public double HIGH_POWER = 1.0;
    public double MED_POWER = 0.5;
    public double LOW_POWER = 0.2;

    public int armDeltaCount = 0; //Need tested value

    public boolean runArmToLevelState = false;

    public int armCurrentArmPositionCount = PICKUP_WHILE_FACING_FORWARD_POSITION; //Default arm position count

    //Constructor`
    public Arm(HardwareMap hardwareMap){
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        initArm();
    }

    //Method is able to initialize the arm
    public void initArm(){
        resetArm();
        turnArmBrakeModeOff();
        armMotorState = ARM_MOTOR_STATE.PICKUP;
        armmotor.setPositionPIDFCoefficients(5.0);
        armmotor.setTargetPosition(PICKUP_WHILE_FACING_FORWARD_POSITION);
        armmotor.setDirection(DcMotorEx.Direction.FORWARD);
    }

    //Turns on the brake for arm motor
    public void turnArmBrakeModeOn(){
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm motor off
    public void turnArmBrakeModeOff() {
        armmotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }



    //Sets arm position to ground junction
    public void moveArmToPickUpWhileTurretFacingForward(){
        //TODO : This should be used only when the Turret is facing forward (to avoid arm hitting the sides of the robot)
        turnArmBrakeModeOn();
        if (SystemState.TurretState == Turret.TURRET_MOTOR_STATE.FACING_FORWARD) {
            armmotor.setTargetPosition(PICKUP_WHILE_FACING_FORWARD_POSITION);
            runArmToLevelState = true;
        }
    }

    //TODO: Set arm position when below Low junction angle dynamically to avoid hitting side of the robot
    public void moveArmToPickUpWhileDynamicTurretAngle(double turretAngle){
        pickupArmWhileDynamicTurretPosition = 0; // TODO: Update with formula
        armmotor.setTargetPosition(pickupArmWhileDynamicTurretPosition);
        runArmToLevelState = true;
    }

    //Sets arm position to low junction
    public void moveArmToLowJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(LOW_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    //Sets arm position or mid junction
    public void moveArmToMidJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(MEDIUM_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    //Sets arm position to high junction
    public void moveArmToHighJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(HIGH_JUNCTION_POSITION);
        runArmToLevelState = true;
    }

    public void modifyArmLength(double joyStickValue){
        armCurrentArmPositionCount = armmotor.getCurrentPosition();
        if (joyStickValue > 0.2) {
            armDeltaCount = (int) (Math.pow((joyStickValue * 1.25 - 0.25), 3) * ARM_DELTA_COUNT_MAX);
            //maxExtended = (ROBOT_HEIGHT - 2)/Math.cos(getShoulderPositionCount * CONVERSION_FACTOR_TO_DEGREES) - F; Algorithm to not hit the ground
        } else if (joyStickValue < -0.2) { //TODO - Convert MIN_RETRACTED to a varible value when shoulder angle < 0, use auto retraction
            armDeltaCount = (int) (Math.pow((joyStickValue * 1.25 + 0.25), 3) * ARM_DELTA_COUNT_MAX);
        } else {
            armDeltaCount = 0;
        }
        if ((armDeltaCount !=0)
                && (armCurrentArmPositionCount >= PICKUP_WHILE_FACING_FORWARD_POSITION)
                && (armCurrentArmPositionCount <= MAX_EXTENDED_POSITION)){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = (int) (armCurrentArmPositionCount + joyStickValue * ARM_DELTA_COUNT_MAX);
            if (armCurrentArmPositionCount < PICKUP_WHILE_FACING_FORWARD_POSITION
                    && SystemState.TurretState == Turret.TURRET_MOTOR_STATE.FACING_FORWARD) {
                armCurrentArmPositionCount = PICKUP_WHILE_FACING_FORWARD_POSITION;
                armMotorState = ARM_MOTOR_STATE.PICKUP;
            } else if (armCurrentArmPositionCount > MAX_EXTENDED_POSITION) {
                armCurrentArmPositionCount = MAX_EXTENDED_POSITION;
                armMotorState = ARM_MOTOR_STATE.MAX_EXTENDED;
            } else {
                armMotorState = ARM_MOTOR_STATE.RANDOM;
            }
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runArmToLevelState = true;
        }
    }

    public double calculateMaxExtensionArmEncoderPositionBasedOnShoulderAngle(){
        double maxExtensionArmEncoderPositionBasedOnShoulderAngle = 0; //TODO: Measure arm max extension in mm

        //TODO: if shoulderAngle > Threshold when arm at full extension will touch the ground
        maxExtensionArmEncoderPositionBasedOnShoulderAngle = MAX_EXTENDED_POSITION;



        return maxExtensionArmEncoderPositionBasedOnShoulderAngle;
    }

    public int convertMotorEncoderValueToArmLength(){
        int convertedMotorEncoderValueToArmLength = 0; //TODO: From encoder value Find Max length and write proportional convertion algorithm
        return convertedMotorEncoderValueToArmLength;
    }


    //sets the arm motor power
    public void runArmToLevel(double power){
        armmotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true){
            armmotor.setPower(power);
            runArmToLevelState = false;
        } else{
            armmotor.setPower(0.0);
        }
    }

    //Resets the arm
    public void resetArm(){
        DcMotorEx.RunMode runMode = armmotor.getMode();
        armmotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setMode(runMode);
    }

    //Returns the current arm position
    public int getArmPositionCount(){
        return armmotor.getCurrentPosition();
    }
}





