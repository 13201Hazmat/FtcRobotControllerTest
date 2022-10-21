package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */
public class Arm {
    //Initialization of armmotor
    public DcMotorEx armmotor;


    //Arm states, either fully extended or retracted all the way
    public enum ARM_STATE{
        MAX_EXTENDED,
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
    public int armmotorBaselineEncoderCount = 0;//Need tested values
    public int armCurrentArmPositionCount = GROUND_JUNCTION; //Default arm position count
    public double max_extended= 0; //Need tested values

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
        armmotor.setTargetPosition(GROUND_JUNCTION + armmotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.GROUND_JUNCTION;
    }

    //Sets arm position to low junction
    public void moveToArmLowJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(LOW_JUNCTION + armmotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.LOW_JUNCTION;
    }

    //Sets arm position or mid junction
    public void moveToArmMidJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(MEDIUM_JUNCTION + armmotorBaselineEncoderCount);
        runArmToLevelState = true;
        previousArmPosition = currentArmPosition;
        currentArmPosition = armMotorPosition.MEDIUM_JUNCTION;
    }

    //Sets arm position to high junction
    public void moveToArmHighJunction(){
        turnArmBrakeModeOn();
        armmotor.setTargetPosition(HIGH_JUNCTION + armmotorBaselineEncoderCount);
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
        armDeltaCount = (int) (Math.pow((joystickAmount - 0.2), 3) * MAX_DELTA);
        if (armCurrentArmPositionCount > RETRACTED && armCurrentArmPositionCount >= GROUND_JUNCTION + armDeltaCount){
            turnArmBrakeModeOn();
            armCurrentArmPositionCount = armCurrentArmPositionCount - armDeltaCount;
            armmotor.setTargetPosition(armCurrentArmPositionCount);
            runArmToLevelState = true;
        }
    }

    //extends the arm for the joystick control
    public void extendArm( /* getShoulderPositionCount */ double joystickAmount ){
        armDeltaCount = (int) (Math.pow((joystickAmount - 0.2), 3) * MAX_DELTA);
        //max_extended = (ROBOT_HEIGHT - 2)/Math.cos(getShoulderPositionCount * CONVERSION_FACTOR_TO_DEGREES) - F; Algorithm to not hit the ground
        if (armCurrentArmPositionCount < max_extended && armCurrentArmPositionCount <= GROUND_JUNCTION - armDeltaCount){
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



