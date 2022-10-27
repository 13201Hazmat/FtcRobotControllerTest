package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
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
    public enum ARM_STATE{
        maxExtended,
        RETRACTED
    }

    //Initialization of ARM_MOTOR_POSITION
    public enum ARM_MOTOR_POSITION{
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        PARKED

    }

    //Initialization of ARM_MOTOR_POSITION and ARM_STATE enums
    public ARM_MOTOR_POSITION armMotorPosition;
    public ARM_STATE armState;


    //Constants for Arm positions
    public static final int RETRACTED= 0; //Need tested values
    public static final int GROUND_JUNCTION= 0; //Need tested values
    public static final int LOW_JUNCTION= 0; //Need tested values
    public static final int MEDIUM_JUNCTION= 0; //Need tested values
    public static final int HIGH_JUNCTION= 0; //Need tested values
    public static final int PARKED = 0; //Need tested values
    public static final int AUTO_RETRACTION_AMOUNT = 0; //need tested values
    public static final int MAX_DELTA = 0; //need tested values

    //Different constants of arm speed
    public static final double HIGH_POWER = 1.0;
    public static final double MED_POWER = 0.5;
    public static final double LOW_POWER = 0.2;

    public int armDeltaCount = 0; //Need tested value
    public ARM_MOTOR_POSITION currentArmPosition = ARM_MOTOR_POSITION.PARKED;
    public ARM_MOTOR_POSITION previousArmPosition = ARM_MOTOR_POSITION.PARKED;
    public boolean runArmToLevelState = false;
    public int armMotorBaselineEncoderCount = 0;//Need tested values
    public int armCurrentArmPositionCount = GROUND_JUNCTION; //Default arm position count
    public double maxExtended= 0; //Need tested values

    //Constructor
    public Arm(HardwareMap hardwareMap){
        armmotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        initArm();
    }

    //Method is able to initialize the arm
    public void initArm(){
        resetArm();
        turnArmBrakeModeOff();
        armmotor.setPositionPIDFCoefficients(5.0);
        armmotor.setTargetPosition(PARKED);
        armmotor.setDirection(DcMotor.Direction.FORWARD);
        currentArmPosition = armMotorPosition.PARKED;
        previousArmPosition = armMotorPosition.PARKED;
    }

    //Turns on the brake for arm
    public void turnArmBrakeModeOn(){
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm off
    public void turnArmBrakeModeOff() {
        armmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //Sets arm poision to ground junction
    public void moveToArmGroundJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(GROUND_JUNCTION + armMotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.GROUND_JUNCTION;
    }

    //Sets arm position to low junction
    public void moveToArmLowJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(LOW_JUNCTION + armMotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.LOW_JUNCTION;
    }

    //Sets arm position or mid junction
    public void moveToArmMidJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(MEDIUM_JUNCTION + armMotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.MEDIUM_JUNCTION;
    }

    //Sets arm position to high junction
    public void moveToArmHighJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(HIGH_JUNCTION + armMotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.HIGH_JUNCTION;
    }

    //sets the arm power
    public void runArmToLevel(double power){
        armmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runArmToLevelState == true){
            armmotor.setPower(power);
            runArmToLevelState = false;
        } else{
            armmotor.setPower(0.0);
        }
    }

    //retracts the arm for joystick control
    public void retractArm(double joystickAmount){
        //armDeltaCount = (int) (Math.pow((joystickAmount * 1.25 - 0.25), 3) * MAX_DELTA); Function is normalized 0.2-1 to 0-1
        if (armCurrentArmPositionCount > RETRACTED && armCurrentArmPositionCount >= GROUND_JUNCTION + armDeltaCount){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount - armDeltaCount;
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runArmToLevelState = true;
        }
    }

    //extends the arm for the joystick control
    public void extendArm( /* getShoulderPositionCount */ double joystickAmount ){
        //armDeltaCount = (int) (Math.pow((joystickAmount * 1.25 - 0.25), 3) *  MAX_DELTA); Function is normalized 0.2-1 to 0-1
        //maxExtended = (ROBOT_HEIGHT - 2)/Math.cos(getShoulderPositionCount * CONVERSION_FACTOR_TO_DEGREES) - F; Algorithm to not hit the ground
        if (armCurrentArmPositionCount < maxExtended && armCurrentArmPositionCount <= GROUND_JUNCTION - armDeltaCount){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount + armDeltaCount;
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runArmToLevelState = true;
        } else{
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount - AUTO_RETRACTION_AMOUNT;
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runArmToLevelState = true;

        }
    }

    //Resets the arm
    public void resetArm(){
        DcMotor.RunMode runMode = armmotor.getMode();
        armmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armmotor.setMode(runMode);
    }

    //Returns the current arm position
    public int getArmPositionCount(){
        return armmotor.getCurrentPosition();
    }
}

//change delta funciton to be exponential in proportion to the joystick, put threshold on joystick(0.2), exponential 0.2-1



