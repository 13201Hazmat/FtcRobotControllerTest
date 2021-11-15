package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
public class Elevator {

    public DcMotorEx elevatorMotor = null;

    public enum ELEVATOR_STATE {
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }

    public ELEVATOR_STATE elevatorState = ELEVATOR_STATE.LEVEL_0;

    // Encoder values for 5203 Gobilda 312rpm motor yyyy encoder counts / revolution
    //5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Encoder Resolution: 537.7 PPR at the Output Shaft
    //https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-19-2-1-ratio-24mm-length-8mm-rex-shaft-312-rpm-3-3-5v-encoder/

    public static int baselineEncoderCount = 0;
    public static int ELEVATOR_LEVEL0_POSITION_COUNT = 0;
    public static int ELEVATOR_LEVEL1_POSITION_COUNT = -100;
    public static int ELEVATOR_LEVEL2_POSITION_COUNT = -600;
    public static int ELEVATOR_LEVEL3_POSITION_COUNT = -1500;
    public static int ELEVATOR_LEVELMAX_POSITION_COUNT = -1550;
    public static int ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT = -100;
    public static int ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT = -100;

    public int elevatorPositionCount = ELEVATOR_LEVEL0_POSITION_COUNT;

    public static double POWER_GOING_UP = 0.7;
    public static double POWER_COMING_DOWN = 0.3;

    public enum ELEVATOR_BUTTON_STATE {
        ON,
        OFF
    }
    public ELEVATOR_BUTTON_STATE elevatorButtonState;

    public Elevator(HardwareMap hardwareMap) {
        elevatorMotor =  hardwareMap.get(DcMotorEx.class, "elevator_motor");;
    }

    /**
     * Initialization for the Elevator
     */
    public void initElevator(){
        elevatorMotor.setPositionPIDFCoefficients(5.0);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetElevator();
        moveElevatorLevel0Position();
        turnElevatorBrakeModeOff();
    }

    /**
     * Reset Elevator Encoder
     */
    public void resetElevator(){
        DcMotor.RunMode runMode = elevatorMotor.getMode();
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotor.setMode(runMode);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOn(){
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevtor brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnElevatorBrakeModeOff(){
        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    public boolean runElevatorToLevelState = false;
    public double motorPowerToRun = POWER_GOING_UP;


    /**
     * Method to run motor to set to the set position
     */
    public void runElevatorToLevel(double power){
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runElevatorToLevelState == true || elevatorMotor.isBusy() == true){
            elevatorMotor.setPower(power);
            runElevatorToLevelState = false;
        } else {
            elevatorMotor.setPower(0.0);
        }
    }

    /**
     * Move Elevator to Level 0
     */
    public void moveElevatorLevel0Position() {
        turnElevatorBrakeModeOff();
        elevatorPositionCount = ELEVATOR_LEVEL0_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        //motorPowerToRun = POWER_GOING_UP;
        setElevatorMotorPowerToRun();
        runElevatorToLevelState = true;
        elevatorState = ELEVATOR_STATE.LEVEL_0;
    }

    /**
     * Move Elevator to Level 1
     */
    public void moveElevatorLevel1Position() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = ELEVATOR_LEVEL1_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        //motorPowerToRun = POWER_GOING_UP;
        setElevatorMotorPowerToRun();
        runElevatorToLevelState = true;
        elevatorState = ELEVATOR_STATE.LEVEL_1;
    }

    /**
     * Move Elevator to Level 2
     */
    public void moveElevatorLevel2Position() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = ELEVATOR_LEVEL2_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        //motorPowerToRun = POWER_GOING_UP;
        setElevatorMotorPowerToRun();
        runElevatorToLevelState = true;
        elevatorState = ELEVATOR_STATE.LEVEL_2;
    }

    /**
     * Move Elevator to Level 2
     */
    public void moveElevatorLevel3Position() {
        turnElevatorBrakeModeOn();
        elevatorPositionCount = ELEVATOR_LEVEL3_POSITION_COUNT + baselineEncoderCount;
        elevatorMotor.setTargetPosition(elevatorPositionCount);
        //motorPowerToRun = POWER_GOING_UP;
        setElevatorMotorPowerToRun();
        runElevatorToLevelState = true;
        elevatorState = ELEVATOR_STATE.LEVEL_3;
    }

    /**
     * Move Elevator Slightly Down
     */
    public void moveElevatorSlightlyDown(){
        if ((elevatorPositionCount <=ELEVATOR_LEVEL3_POSITION_COUNT) &&
                elevatorPositionCount >= ELEVATOR_LEVEL1_POSITION_COUNT + ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT){
            turnElevatorBrakeModeOn();
            elevatorPositionCount = elevatorPositionCount - ELEVATOR_DELTA_SLIGHTLY_DOWN_DELTA_COUNT;
            elevatorMotor.setTargetPosition(elevatorPositionCount);
            motorPowerToRun = POWER_COMING_DOWN;
            runElevatorToLevelState = true;
        }
    }

    /**
     * Move Elevator Slightly Up
     */
    public void moveElevatorSlightlyUp(){
        if ((elevatorPositionCount > ELEVATOR_LEVEL0_POSITION_COUNT) &&
                elevatorPositionCount <= ELEVATOR_LEVELMAX_POSITION_COUNT - ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT){
            turnElevatorBrakeModeOn();
            elevatorPositionCount = elevatorPositionCount + ELEVATOR_DELTA_SLIGHTLY_UP_DELTA_COUNT;
            elevatorMotor.setTargetPosition(elevatorPositionCount);
            motorPowerToRun = POWER_GOING_UP;
            runElevatorToLevelState = true;
        }
    }

    /**
     * Returns Intake motor state
     */
    public ELEVATOR_STATE getElevatorState() {
        return elevatorState;
    }

    /**
     * Returns Intake motor state
     */
    public int getElevatorPositionCount() {
        return elevatorPositionCount;
    }

    public void setElevatorMotorPowerToRun(){
        if (elevatorMotor.getCurrentPosition() > elevatorMotor.getTargetPosition()) {
            motorPowerToRun = POWER_GOING_UP;
        } else {
            motorPowerToRun = POWER_COMING_DOWN;
        }
    }


}