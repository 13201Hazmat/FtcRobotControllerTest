package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.MinorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;

/**
 * Defenition of the AutoControl Class <BR>
 *
 * HzAutoControl consists of methods to control the robot subsystems in autonomous mode <BR>
 * This is set up as a state machine. And replicates all commands as in the gamepad <BR>
 *
 */

public class AutonomousController {

    //Create gamepad object reference to connect to gamepad1
    public DriveTrain driveTrain;
    public Intake intake;
    public Elevator elevator;
    public Magazine magazine;
    public Spinner spinner;
    public MajorArm majorArm;
    public MinorArm minorArm;

    public Pose2d startPose = GameField.BLUE_WAREHOUSE_STARTPOS;

    // TODO: Declare autonomous option logic based on key pad selection
    /* Example
    public boolean launchHighGoalOrPowerShot = false;
    public boolean dropFirstWobbleGoal = false;
    public boolean pickRingFromTargetMarker = false;
    public boolean launchRingsPickedFromTargetMarkerToHighGoal = false;
    public boolean pickAndDropSecondWobbleGoal = false;
         */

    /**
     * Constructor for HzGamepad1 class that extends gamepad.
     * Assign the gamepad1 given in OpMode to the gamepad used here.
     *
     * TODO: Add more subsystems in declaration
     */
    public AutonomousController(DriveTrain driveTrain,
                                Intake intake,
                                Elevator elevator,
                                Magazine magazine,
                                Spinner spinner,
                                MajorArm majorArm,
                                MinorArm minorArm) {
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.elevator = elevator;
        this.magazine = magazine;
        this.spinner = spinner;
        this.majorArm = majorArm;
        this.minorArm = minorArm;
    }

    /**
     * Main control function that calls the runs of each of the subsystems in sequence
     */
    public void runAutoControl(){
        int counter = 0;
        while (counter < 5) {
            runAutoIntake();
            runAutoElevator();
            runAutoMagazine();
            runAutoSpinner();
            runAutoMajorArm();
            counter++;
        }
    }

    /**
     * Intake Commands :
     *      startAutoIntake()
     *      stopAutoIntake()
     */

    enum AUTO_INTAKE_STATE{
        RUNNING,
        STOPPED,
    }
    AUTO_INTAKE_STATE autoIntakeState = AUTO_INTAKE_STATE.STOPPED;

    /**
     * Starts the autonomous intake
     */
    public void startAutoIntake(){
        autoIntakeState = AUTO_INTAKE_STATE.RUNNING;
        runAutoControl();
    }

    /**
     * Stops Autonomous intake
     */
    public void stopAutoIntake(){
        autoIntakeState = AUTO_INTAKE_STATE.STOPPED;
        runAutoControl();
    }

    /**
     * Runs the Intake with protection on the elevator level state and magazine state
     */
    public void runAutoIntake() {
        if (autoIntakeState == AUTO_INTAKE_STATE.RUNNING &&
                elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0) {
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.COLLECT) {
                magazine.moveMagazineToCollect();
            }
            intake.startIntakeMotorInward();
        } else { //(autoIntakeState == AUTO_INTAKE_STATE.STOPPED)
            intake.stopIntakeMotor();
        }
    }

    /**
     * Elevator Commands :
     *      moveAutoElevatorLevel0()
     *      moveAutoElevatorLevel1()
     *      moveAutoElevatorLevel2()
     *      moveAutoElevatorLevel3()
     */

    enum  AUTO_ELEVATOR_STATE{
        LEVEL_0,
        LEVEL_1,
        LEVEL_2,
        LEVEL_3,
    }
    AUTO_ELEVATOR_STATE autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_0;

    /**
     * Moves elevator level to zero
     */
    public void moveAutoElevatorLevel0(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_0;
        runAutoControl();
    }

    /**
     * Moves elevator to level one
     */
    public void moveAutoElevatorLevel1(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_1;
        runAutoControl();
    }

    /**
     * Moves elevator to level two
     */
    public void moveAutoElevatorLevel2(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_2;
        runAutoControl();
    }

    /**
     * Moves elevator to level three
     */
    public void moveAutoElevatorLevel3(){
        autoElevatorState = AUTO_ELEVATOR_STATE.LEVEL_3;
        runAutoControl();
    }

    /**
     * Runs the elevator to a specific level based on the state of the magazine
     */
    public void runAutoElevator() {
        if (autoElevatorState == AUTO_ELEVATOR_STATE.LEVEL_0){
            elevator.moveElevatorLevel0Position();
            autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;

        } else {
            autoIntakeState = AUTO_INTAKE_STATE.STOPPED;
            //autoMagazineState = AUTO_MAGAZINE_STATE.TRANSPORT;
            switch (autoElevatorState){
                case LEVEL_1:
                    elevator.moveElevatorLevel1Position();
                    break;
                case LEVEL_2:
                    elevator.moveElevatorLevel2Position();
                    break;
                case LEVEL_3:
                    elevator.moveElevatorLevel3Position();
                    break;
            }
        }

        if (elevator.runElevatorToLevelState){
            elevator.runElevatorToLevel(elevator.motorPowerToRun);
        }

    }

    /**
     * Magazine Commands :
     *      moveAutoMagazineToCollect()
     *      moveAutoMagazineToTransport()
     *      moveAutoMagazineToDrop()
     */
    enum AUTO_MAGAZINE_STATE{
        COLLECT,
        TRANSPORT,
        DROP,
    }
    AUTO_MAGAZINE_STATE autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;

    /**
     * moves magazine to Collect mode
     */
    public void moveAutoMagazineToCollect(){
        autoMagazineState = AUTO_MAGAZINE_STATE.COLLECT;
        runAutoControl();
    }

    /**
     * Moves magazine to Transport Mode
     */
    public void moveAutoMagazineToTransport(){
        autoMagazineState = AUTO_MAGAZINE_STATE.TRANSPORT;
        runAutoControl();
    }

    /**
     * Moves magazine to Drop mode
     */
    public void moveAutoMagazineToDrop(){
        autoMagazineState = AUTO_MAGAZINE_STATE.DROP;
        runAutoControl();
    }

    /**
     * Changes magazine state depending if the color sensor is sensing a freight
     */
    public void runAutoMagazine() {
        if (magazine.magazineColorSensor instanceof SwitchableLight) {
            if (elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0 &&
                    magazine.getMagazineColorSensorState() == Magazine.MAGAZINE_COLOR_SENSOR_STATE.EMPTY) {
                ((SwitchableLight) magazine.magazineColorSensor).enableLight(true);
            } else {
                ((SwitchableLight) magazine.magazineColorSensor).enableLight(false);
            }
        }
        switch (autoMagazineState){
            case TRANSPORT:
                magazine.moveMagazineToTransport();
                break;
            case DROP :
                magazine.moveMagazineToDrop();
                break;
            case COLLECT:
                magazine.moveMagazineToCollect();
                break;
        }
    }

    /**
     * Major Arm Commands :
     *      moveAutoMajorArmPickup
     *      moveAutoMajorArmPark
     *      openAutoMajorClaw
     *      closeAutoMajorClaw
     *      runAutoMajorArm
     */

    enum AUTO_MAJOR_ARM_STATE{
        PICKUP,
        PARKED,
    }
    AUTO_MAJOR_ARM_STATE autoMajorArmState = AUTO_MAJOR_ARM_STATE.PARKED;

    /**
     * moves major arm to pickup position
     */
    public void moveAutoMajorArmPickup(){
        autoMajorArmState = AUTO_MAJOR_ARM_STATE.PICKUP;
        runAutoControl();
    }

    /**
     * moves major arm to park position
     */
    public void moveAutoMajorArmPark(){
        autoMajorArmState = AUTO_MAJOR_ARM_STATE.PARKED;
        runAutoControl();
    }
    public enum AUTO_MAJOR_CLAW_STATE {
        OPEN,
        CLOSED,
    }
    AUTO_MAJOR_CLAW_STATE autoMajorClawState = AUTO_MAJOR_CLAW_STATE.CLOSED;

    /**
     * Opens the major claw
     */
    public void openAutoMajorClaw(){
        autoMajorClawState = AUTO_MAJOR_CLAW_STATE.OPEN;
        runAutoControl();
    }

    /**
     * Closes major claw
     */
    public void closeAutoMajorClaw(){
        autoMajorClawState = AUTO_MAJOR_CLAW_STATE.CLOSED;
        runAutoControl();
    }

    /**
     * Sets the major arm to level state and claw state
     */
    public void runAutoMajorArm() {
        if(autoMajorArmState == AUTO_MAJOR_ARM_STATE.PARKED){
            majorArm.moveMajorArmParkingPosition();
        } else {
            majorArm.moveMajorArmPickupPosition();
        }

        if (majorArm.runArmToLevelState) {
            majorArm.runMajorArmToLevel(majorArm.ARM_MOTOR_POWER);
        }

        if(autoMajorClawState == AUTO_MAJOR_CLAW_STATE.OPEN){
            majorArm.openMajorClaw();
        } else{
            majorArm.closeMajorClaw();
        }

    }

    /**
     * Spinner Commands :
     *      runAutoSpinner
     */

    public enum AUTO_SPINNER_STATE{
        CLOCKWISE,
        ANTICLOCKWISE,
        STOPPED,
    }

    public AUTO_SPINNER_STATE autoSpinnerState = AUTO_SPINNER_STATE.STOPPED;

    /**
     * Runs the spinner in a specific direction
     */
    public void runAutoSpinner() {
        switch (autoSpinnerState) {
            case ANTICLOCKWISE:
                spinner.runSpinnerMotorAnticlockwise();
                break;
            case CLOCKWISE:
                spinner.runSpinnerMotorClockwise();
                break;
            case STOPPED:
                spinner.stopSpinnerMotor();
                break;
        }
    }
}