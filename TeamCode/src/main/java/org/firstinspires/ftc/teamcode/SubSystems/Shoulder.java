package org.firstinspires.ftc.teamcode.SubSystems;
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

public class Shoulder {
    //Initialization of <Fill>
    public DcMotorEx rightShoulderMotor, leftShoulderMotor;

    //Initialization of <Fill>
    public enum SHOULDER_STATE {
        PICKUP, //PARKED, MIN_POSITION,
        GROUND_JUNCTION,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        MAX_RAISED
    }

    //Initialization of <Fill>
    public SHOULDER_STATE shoulderState;

    public boolean runShoulderToLevelState = false;
    public int SHOULDER_DELTA_COUNT_MAX = 50;
    public int shoulderDeltaCount = 0; //Need tested value


    public int shoulderPositionCount = GROUND_JUNCTION_POSITION; //Default shoulder position count

    public static final int PICKUP_POSITION = 0;
    public static final int GROUND_JUNCTION_POSITION = 200; //Need tested values
    public static final int LOW_JUNCTION_POSITION = 400; //need tested values
    public static final int MEDIUM_JUNCTION_POSITION = 600; //need tested values
    public static final int HIGH_JUNCTION_POSITION = 800; //need tested values
    public static final double MAX_RAISED = 3000; //Need tested values


    //Different constants of shoulder speed
    public double HIGH_POWER = 1.0;
    public double MED_POWER = 0.5;
    public double LOW_POWER = 0.2;

    //Constructor
    public Shoulder(HardwareMap hardwareMap){
        leftShoulderMotor = hardwareMap.get(DcMotorEx.class, "lshmotor");
        rightShoulderMotor = hardwareMap.get(DcMotorEx.class, "rshmotor");
        initShoulder();
    }

    //Method is able to <Fill>
    public void initShoulder(){
        resetShoulder();
        turnShoulderBrakeModeOff();
        leftShoulderMotor.setPositionPIDFCoefficients(5.0);
        rightShoulderMotor.setPositionPIDFCoefficients(5.0);
        leftShoulderMotor.setTargetPosition(PICKUP_POSITION);
        rightShoulderMotor.setTargetPosition(PICKUP_POSITION);
        leftShoulderMotor.setDirection(DcMotorEx.Direction.FORWARD);
        rightShoulderMotor.setDirection(DcMotorEx.Direction.REVERSE);
    }

    public void resetShoulder(){
        DcMotorEx.RunMode runMode = leftShoulderMotor.getMode();
        leftShoulderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShoulderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShoulderMotor.setMode(runMode);
        rightShoulderMotor.setMode(runMode);

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

    public void turnShoulderBrakeModeOn() {
        leftShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void turnShoulderBrakeModeOff() {
        leftShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        rightShoulderMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //gets the position of the shoulder
    public int getShoulderPositionCount() {
        return rightShoulderMotor.getCurrentPosition();
    }

    //Sets shoulder position to ground junction
    public void moveToShoulderPickup() {
        turnShoulderBrakeModeOff();
        leftShoulderMotor.setTargetPosition(GROUND_JUNCTION_POSITION);
        rightShoulderMotor.setTargetPosition(GROUND_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }

    //Sets shoulder position to low junction
    public void moveToShoulderLowJunction() {
        turnShoulderBrakeModeOn();
        leftShoulderMotor.setTargetPosition(LOW_JUNCTION_POSITION);
        rightShoulderMotor.setTargetPosition(LOW_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }

    //Sets shoulder position or mid junction
    public void moveToShoulderMidJunction() {
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setTargetPosition(MEDIUM_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition(MEDIUM_JUNCTION_POSITION);
        runShoulderToLevelState = true;

    }

    //Sets shoulder position to high junction
    public void moveToShoulderHighJunction() {
        turnShoulderBrakeModeOn();
        rightShoulderMotor.setTargetPosition(HIGH_JUNCTION_POSITION);
        leftShoulderMotor.setTargetPosition(HIGH_JUNCTION_POSITION);
        runShoulderToLevelState = true;
    }


    public void lowerShoulder(double leftTriggerAmount) {
        turnShoulderBrakeModeOn();
        shoulderDeltaCount = (int) (Math.pow((leftTriggerAmount * 1.25 - 0.25), 3) * SHOULDER_DELTA_COUNT_MAX);
        if (shoulderPositionCount > PICKUP_POSITION + shoulderDeltaCount){

            shoulderPositionCount = shoulderPositionCount - shoulderDeltaCount;
        }else{

            shoulderPositionCount = PICKUP_POSITION;
            turnShoulderBrakeModeOff();
        }
        rightShoulderMotor.setTargetPosition(shoulderPositionCount);
        leftShoulderMotor.setTargetPosition(shoulderPositionCount);
        runShoulderToLevelState = true;
    }

    public void raiseShoulder(double rightTriggerAmount) {
        shoulderDeltaCount = (int) (Math.pow((rightTriggerAmount * 1.25 - 0.25), 3) * SHOULDER_DELTA_COUNT_MAX);

        if (shoulderPositionCount < MAX_RAISED){
            turnShoulderBrakeModeOn();
            shoulderPositionCount = shoulderPositionCount + shoulderDeltaCount;
            rightShoulderMotor.setTargetPosition(shoulderPositionCount);
            leftShoulderMotor.setTargetPosition(shoulderPositionCount);
            runShoulderToLevelState = true;
        }
    }


}
