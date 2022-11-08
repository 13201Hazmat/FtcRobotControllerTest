package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
 *     extendArm and retractArm functions extend and retract arm based on a delta value determined <BR>
 *     by the joystick <BR>
 *     resetArm resets the arm to the original position and states <BR>
 *     runArmToLevel runs the arm to the levels determined by the other functions <BR>
 */

public class Arm {
    //Initialization of armmotor
    public DcMotorEx armMotor;

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
    public ARM_MOTOR_STATE armMotorState;

    //Constants for Arm Standard positions
    public static final int PICKUP_WHILE_FACING_FORWARD_POSITION = 0;
    public static final int GROUND_JUNCTION_WHILE_FACING_FORWARD_POSITION = 0;
    public static final int LOW_JUNCTION_POSITION = (int) 1000;
    public static final int MEDIUM_JUNCTION_POSITION = (int) 2000;
    public static final int HIGH_JUNCTION_POSITION = (int) 3000;
    public static final int MAX_EXTENDED_POSITION = (int) 4500;
    public int armCurrentPosition = PICKUP_WHILE_FACING_FORWARD_POSITION; //Default arm position count
    public int armNewPosition = PICKUP_WHILE_FACING_FORWARD_POSITION;

    public int pickupArmWhileDynamicTurretPosition = 0;

    public static final int ARM_DELTA_COUNT_MAX = 200; //need tested values

    //Different constants of arm speed
    public static final double ARM_POWER = 0.5;

    public int armDeltaCount = 0; //Need tested value

    public boolean runArmToLevelState = false;

    //Constructor`
    public Arm(HardwareMap hardwareMap){
        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        initArm();
    }

    //Method is able to initialize the arm
    public void initArm(){
        resetArm();
        turnArmBrakeModeOff();
        armMotorState = ARM_MOTOR_STATE.PICKUP;
        armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        armMotor.setPositionPIDFCoefficients(5.0); //TODO: Adjust so that it does not shake when raised
        armMotor.setTargetPosition(PICKUP_WHILE_FACING_FORWARD_POSITION);
        armMotor.setDirection(DcMotorEx.Direction.REVERSE); //TODO: Check direction of robot
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

    public int convertMotorEncoderValueToArmLength(){
        int convertedMotorEncoderValueToArmLength = 0; //TODO: From encoder value Find Max length and write proportional convertion algorithm
        return convertedMotorEncoderValueToArmLength;
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
}





