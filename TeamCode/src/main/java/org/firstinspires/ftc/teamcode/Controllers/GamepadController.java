package org.firstinspires.ftc.teamcode.Controllers;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
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
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection 
 * made on gamepads <BR>
 * 
 * For Hazmat Freight Frenzy, two Gamepads are used (gamepad1 and gamepad2) <BR>
 *
 * The controls are as follows: (replace with gamepad2 for 2nd gamepad) <BR>
 *      <emsp>Left Stick for pan motion (gamepad1.left_stick_x and gamepad2.left_stick_y) <BR>
 *      <emsp>Right Stick for turn motion (gamepad2.right_stick_x: gamepad1.right_stick_y) <BR>
 *      <emsp>Right Bumper magazine flip and majorClaw state(gp2) (gamepad1.right_bumper, gamepad2.right_bumper) <BR>
 *      <emsp>Left Bumper for spinner state and minorArm state(gp2) (gamepad1.left_bumper, gamepad2.left_bumper) <BR>
 *      <emsp>Right Trigger for turbo, and majorArm Parking position(gp2) (gamepad1.right_trigger, gamepad2.right_trigger) <BR>
 *      <emsp>Button A for elevator intake level and major arm pickup position(gp2) (gamepad1.a, gamepad2.a) <BR>
 *      <emsp>Button Y for elevator level 2 and major arm capstone position(gp2) (gamepad1.y, gamepad2.y) <BR>
 *      <emsp>Button X for elevator level 1 and majorArm down one level(gp2) (gamepad1.x, gamepad2.x) <BR>
 *      <emsp>Button B for elevator level 3 and majorArm level up one(gp2) (gamepad1.b, gamepad2.b) <BR>
 *      <emsp>Button Dpad_up for intake out & stop, also for minorArm level up one (gamepad1.dpad_up, gamepad2.dpad_up) <BR>
 *      <emsp>Button Dpad_down for intake in & stop, also for minorArm level down one (gamepad1.dpad_down, gamepad2.dpad_down) <BR>
 *
 * To access the gamepad functions, use the gp1Get* or gp2Get* functions at the end of this class <BR>
 *     <emsp>gp1GetLeftStickX(), gp2GetLeftStickX()</emsp>
 *     <emsp>gp1GetLeftStickY(), gp2GetLeftStickY()</emsp>
 *     <emsp>gp1GetRightStickX(), gp2GetRightStickX()</emsp>
 *     <emsp>gp1GetRightStickY(), gp2GetRightStickY()</emsp>
 *     <emsp>gp1GetLeftTrigger(), gp2GetRightTrigger()</emsp>
 *     <emsp>gp1GetLeftTriggerPress(), gp2GetRightTriggerPress for toggle value()</emsp>
 *     <emsp>gp1GetLeftBumper(), gp2GetRightBumper()</emsp>
 *     <emsp>gp1GetLeftBumperPress(), gp2GetRightBumperPress for toggle value()</emsp>
 *     <emsp>gp1GetX(), gp2GetY(), gp1GetA(), gp2GetB()</emsp>
 *     <emsp>gp1GetXPress(), gp2GetYPress(), gp1GetAPress(), gp2GetBPress() for toggle value()</emsp>
 *     <emsp>gp1GetDpad_up(), gp2GetDpad_down(). gp1GetDpad_left(), gp2GetDpad_right() </emsp>
 *     <emsp>gp1GetDpad_upPress(), gp2GetDpad_downPress(). gp1GetDpad_leftPress(), gp2GetDpad_rightPress()  for toggle value()</emsp>
 *     <emsp>gp1GetStart(), gp2GetStart()</emsp>
 *
 */

public class GamepadController {

    //Create object reference to objects to systems passed from TeleOp
    public Gamepad hzGamepad1, hzGamepad2;
    public DriveTrain driveTrain;
    public Intake intake;
    public Elevator elevator;
    public Magazine magazine;
    public Spinner spinner;
    public MajorArm majorArm;
    public MinorArm minorArm;

    /**
     * Constructor for HzGamepad1 and HzGamepad2 class that extends gamepad.
     * Assign the gamepad1 and gamepad2 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad hzGamepad1,
                             Gamepad hzGamepad2,
                             DriveTrain driveTrain,
                             Intake intake,
                             Elevator elevator,
                             Magazine magazine,
                             Spinner spinner,
                             MajorArm majorArm,
                             MinorArm minorArm) {
        this.hzGamepad1 = hzGamepad1;
        this.hzGamepad2 = hzGamepad2;
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.elevator = elevator;
        this.magazine = magazine;
        this.spinner = spinner;
        this.majorArm = majorArm;
        this.minorArm = minorArm;
    }

    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runIntake();
        runElevator();
        runMagazine();
        runSpinner();
        runMajorArm();
        runMinorArm();
        runDriveControl_byRRDriveModes();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     */
    // RR Drive Train
    public void runDriveControl_byRRDriveModes() {

        driveTrain.poseEstimate = driveTrain.getPoseEstimate();

        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;

        if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
            if (Math.abs(gp1GetLeftStickX())>0.1 || Math.abs(gp1GetLeftStickY())>0.1) {
                if (elevator.getElevatorState() != Elevator.ELEVATOR_STATE.LEVEL_0) {
                    driveTrain.gamepadInput = new Vector2d(
                            -gp1TurboMode(gp1GetLeftStickY()),
                            -gp1TurboMode(gp1GetLeftStickX()));
                } else {
                    // Avoid using Turbo when elevator is at Level 0
                    driveTrain.gamepadInput = new Vector2d(
                            -limitStick(gp1GetLeftStickY()),
                            -limitStick(gp1GetLeftStickX()));
                }
            } else {
                driveTrain.gamepadInput = new Vector2d(
                        -limitStick(gp2GetLeftStickY()),
                        -limitStick(gp2GetLeftStickX()));
            }
        };

        if (driveTrain.driveType == DriveTrain.DriveType.FIELD_CENTRIC){

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                driveTrain.gamepadInput = new Vector2d(
                        gp1TurboMode(gp1GetLeftStickX()),
                        -gp1TurboMode(gp1GetLeftStickY())
                ).rotated(-driveTrain.poseEstimate.getHeading());
            };

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
                driveTrain.gamepadInput = new Vector2d(
                        -gp1TurboMode(gp1GetLeftStickX()),
                        gp1TurboMode(gp1GetLeftStickY())
                ).rotated(-driveTrain.poseEstimate.getHeading());
            };
        }
        if (Math.abs(gp1GetRightStickX())>0.1) {
            driveTrain.gamepadInputTurn = -gp1TurboMode(gp1GetRightStickX());
        } else {
            driveTrain.gamepadInputTurn = -limitStick(gp2GetRightStickX());
        }

        /*TCode to implement slight left / right turn. Uncomment to use
        if (gp1GetButtonXPress()) {
            hzDrive.augmentedControl = HzDrive.AugmentedControl.TURN_DELTA_LEFT;
        }

        if (gp1GetButtonBPress()) {
            hzDrive.augmentedControl = HzDrive.AugmentedControl.TURN_DELTA_RIGHT;
        }
        */

        driveTrain.driveTrainPointFieldModes();

    }

    //TODO: Add controller code for more subsystems as above
    /**
     * runIntake sets the differnt intake controls, if intake should take in freight(Dpad_downPress) or the intake should run the opposite
     * direction in order for a stuck freight to be out of intake. <BR>
     */
    public void runIntake(){ //this function should be at LaunchController's place after order change
        if (gp1GetDpad_downPress()) {
            if(intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.RUNNING) {
            //&& elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0) {
                if (elevator.getElevatorState() != Elevator.ELEVATOR_STATE.LEVEL_0) {
                    elevator.moveElevatorLevel0Position();
                }
                if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.COLLECT) {
                    magazine.moveMagazineToCollect();
                }
                intake.startIntakeMotorInward();
            } else if(intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.STOPPED) {
                intake.stopIntakeMotor();
            }
        }

        //Reverse Intake motors and run - in case of stuck state)
        if (gp1GetDpad_upPress()) {
            if (intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.REVERSING) {
                intake.startIntakeMotorOutward();
            } else if (intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.STOPPED) {
                intake.stopIntakeMotor();
            }
        }
    }

    /**
     * runElevator sets the different elevator controls, if the elevator should be in a specific level
     * and has protection for drivers for the magazine and the intial level state position.<BR>
       */
    public void runElevator(){ //this function should be at LaunchController's place after order change
        //TODO: Protect turning on grip only when elevator motor is not moving

        if (gp1GetButtonAPress()){
            if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_0) {
                elevator.moveElevatorLevel0Position();
            }
            if (elevator.runElevatorToLevelState){
                elevator.runElevatorToLevel(elevator.motorPowerToRun);
            }
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.COLLECT) {
                magazine.moveMagazineToCollect();
            }
        }

        if (gp1GetButtonXPress()){
            if(intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.RUNNING){
                intake.stopIntakeMotor();
            }
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.TRANSPORT) {
                magazine.moveMagazineToTransport();
            }
            if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_1) {
                elevator.moveElevatorLevel1Position();
            }

        }

        if (gp1GetButtonYPress()){
            if(intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.RUNNING){
                intake.stopIntakeMotor();
            }
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.TRANSPORT) {
                magazine.moveMagazineToTransport();
            }
            if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_2) {
                elevator.moveElevatorLevel2Position();
            }

        }

        if (gp1GetButtonBPress()){
            if(intake.getIntakeMotorState() == Intake.INTAKE_MOTOR_STATE.RUNNING){
                intake.stopIntakeMotor();
            }
            if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.TRANSPORT) {
                magazine.moveMagazineToTransport();
            }
            if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_3) {
                elevator.moveElevatorLevel3Position();
            }

        }

        if (!gp1GetStart()) {
            if (gp1GetLeftTriggerPress()) {
                elevator.moveElevatorSlightlyDown();
            }
        } else {
            if (gp1GetLeftTriggerPress()) {
                elevator.moveElevatorSlightlyUp();
            }
        }

        if (elevator.runElevatorToLevelState){
            elevator.runElevatorToLevel(elevator.motorPowerToRun);
        }
    }

    public enum AUTO_MAGAZINE {
        ON,
        OFF
    }
    public AUTO_MAGAZINE autoMagazine = AUTO_MAGAZINE.ON;

    /**
     * runMagazine sets the different magazine controls, if the magazine should be in transport or
     * drop position depending on the current elevator position and other states. <BR>
     */
    public void runMagazine(){ //this function should be at LaunchController's place after order change
        if (gp1GetRightBumperPress()) {
            if(elevator.getElevatorState() != Elevator.ELEVATOR_STATE.LEVEL_0) {
                if (magazine.getMagazineServoState() == Magazine.MAGAZINE_SERVO_STATE.TRANSPORT) {
                    magazine.moveMagazineToDrop();
                } else if (magazine.getMagazineServoState() == Magazine.MAGAZINE_SERVO_STATE.DROP) {
                    magazine.moveMagazineToTransport();
                }
            } else{
                if (magazine.getMagazineServoState() == Magazine.MAGAZINE_SERVO_STATE.TRANSPORT ) {
                    magazine.moveMagazineToCollect();
                } else if(intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.RUNNING) {
                    intake.stopIntakeMotor();
                    magazine.moveMagazineToTransport();
                }
            }
        }

        if (gp1GetStart() && gp1GetRightTriggerPress()) {
            if (autoMagazine == AUTO_MAGAZINE.ON) {
                autoMagazine = AUTO_MAGAZINE.OFF;
            } else {
                autoMagazine = AUTO_MAGAZINE.ON;
            }
        }

        if (magazine.magazineColorSensor instanceof SwitchableLight) {
            if (elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0 &&
                    magazine.getMagazineColorSensorState() == Magazine.MAGAZINE_COLOR_SENSOR_STATE.EMPTY) {
                ((SwitchableLight) magazine.magazineColorSensor).enableLight(true);
            } else {
                ((SwitchableLight) magazine.magazineColorSensor).enableLight(false);
            }
        }


        if (autoMagazine == AUTO_MAGAZINE.ON) {
            if (elevator.getElevatorState() == Elevator.ELEVATOR_STATE.LEVEL_0) {
                if (magazine.getMagazineColorSensorState() == Magazine.MAGAZINE_COLOR_SENSOR_STATE.LOADED) {
                    if (magazine.getMagazineServoState() != Magazine.MAGAZINE_SERVO_STATE.TRANSPORT) {
                        magazine.moveMagazineToTransport();
                        elevator.moveElevatorLevel1Position();
                        intake.stopIntakeMotor();
                    }
                }
            }
        }
    }

    /**
     * runSpinner sets the differnt spinner controls, if the spinner should be spinning clockwise or
     * anticlockwise or not spinning at all, this functions reads the current state and depending on
     * the state, the function will execute the motion the spinner moves in. <BR>
      */
    public void runSpinner(){ //this function should be at LaunchController's place after order change
        if (!gp1GetStart()) { //Normal condition, start not pressed
            if (gp1GetLeftBumperPress()) {
                //Spinner is running
                if (spinner.getSpinnerMotorState() == Spinner.SPINNER_MOTOR_STATE.CLOCKWISE ||
                        spinner.getSpinnerMotorState() == Spinner.SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                    spinner.stopSpinnerMotor();
                } else {
                    //Spinner not running
                    if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                        if (spinner.getSpinnerMotorState() != Spinner.SPINNER_MOTOR_STATE.CLOCKWISE) {
                            spinner.runSpinnerMotorClockwise();
                        }
                    } else { //if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                        if (spinner.getSpinnerMotorState() != Spinner.SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                            spinner.runSpinnerMotorAnticlockwise();
                        }
                    }
                }
            }
        } else { //Alternate  condition, start pressed
            if (gp1GetLeftBumperPress()) {
                //Spinner is running
                if (spinner.getSpinnerMotorState() == Spinner.SPINNER_MOTOR_STATE.CLOCKWISE ||
                        spinner.getSpinnerMotorState() == Spinner.SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                    spinner.stopSpinnerMotor();
                } else {
                    //Spinner not running
                    if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                        if (spinner.getSpinnerMotorState() != Spinner.SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
                            spinner.runSpinnerMotorAnticlockwise();
                        }
                    } else { //if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                        if (spinner.getSpinnerMotorState() != Spinner.SPINNER_MOTOR_STATE.CLOCKWISE) {
                            spinner.runSpinnerMotorClockwise();
                        }
                    }
                }
            }
        }
    }


    /**
     * runMajorArm sets the different majorArm controls, if the majorArm should be down or up one
     * level or change the arm specifically to a another level by one button press. <BR>
     */
    public void runMajorArm(){ //this function should be at LaunchController's place after order change
        if (gp2GetButtonXPress()){
            majorArm.moveMajorArmDownOne();
        }
        if (gp2GetButtonBPress()) {
            majorArm.moveMajorArmUpOne();
        }
        if(gp2GetButtonAPress()){
            majorArm.moveMajorArmPickupPosition();
        }
        if(gp2GetButtonYPress()){
            majorArm.moveMajorArmCapstonePosition();
        }
        if(gp2GetRightTriggerPress()){
            majorArm.moveMajorArmParkingPosition();
        }

        if (majorArm.runArmToLevelState) {
            majorArm.runMajorArmToLevel(majorArm.ARM_MOTOR_POWER);
        }

        if (!gp2GetStart()) {
            if (gp2GetLeftTriggerPress()) {
                majorArm.moveMajorArmSlightlyDown();
                if (majorArm.runArmToLevelState) {
                    majorArm.runMajorArmToLevel(majorArm.ARM_MOTOR_DELTA_POWER);
                }

            }
        } else {
            if (gp2GetLeftTriggerPress()) {
                majorArm.moveMajorArmSlightlyUp();
                if (majorArm.runArmToLevelState) {
                    majorArm.runMajorArmToLevel(majorArm.ARM_MOTOR_DELTA_POWER);
                }
            }
        }



        if(gp2GetRightBumperPress()){
            majorArm.changeMajorClawState();
        }
    }

    /**
     * runMinorArm sets the different minorArm controls, whether the arm should be up or down
     * one level, or the current claw state. <BR>
     */
    public void runMinorArm() { //this function should be at LaunchController's place after order change
        if (gp2GetDpad_downPress()){
            minorArm.moveMinorArmDownOne();
        }
        if (gp2GetDpad_upPress()) {
            minorArm.moveMinorArmUpOne();
        }
        if(gp2GetLeftBumperPress()){
            minorArm.changeMinorClawState();
        }
    }

    //*********** KEY PAD MODIFIERS BELOW ***********

    //**** Gamepad buttons
    //Records last button press to deal with single button presses doing a certain methods
    boolean gp1ButtonALast = false;
    boolean gp1ButtonBLast = false;
    boolean gp1ButtonXLast = false;
    boolean gp1ButtonYLast = false;
    boolean gp1RightBumperLast = false;
    boolean gp1LeftBumperLast = false;
    boolean gp1Dpad_upLast = false;
    boolean gp1Dpad_downLast = false;
    boolean gp1Dpad_leftLast = false;
    boolean gp1Dpad_rightLast = false;
    boolean gp1LeftTriggerLast = false;
    boolean gp1RightTriggerLast = false;

    boolean gp2ButtonALast = false;
    boolean gp2ButtonBLast = false;
    boolean gp2ButtonXLast = false;
    boolean gp2ButtonYLast = false;
    boolean gp2RightBumperLast = false;
    boolean gp2LeftBumperLast = false;
    boolean gp2Dpad_upLast = false;
    boolean gp2Dpad_downLast = false;
    boolean gp2Dpad_leftLast = false;
    boolean gp2Dpad_rightLast = false;
    boolean gp2LeftTriggerLast = false;
    boolean gp2RightTriggerLast = false;

    /**
     * Method to convert linear map from gamepad1 and gamepad2 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.33); //0.25
    }

    /**
     * Method to implement turbo speed mode - from reduced speed of 25% of cubic factor to
     * 100% speed, but controlled by acceleration of the force of pressing the Right Tigger.
     *
     * @param stickInput input value of button stick vector
     * @return modified value of button stick vector
     */
    public double gp1TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp1GetRightTrigger();
        //acceleration_factor = 1.0 + 3.0 * rightTriggerValue;
        acceleration_factor = 1.0 + 2.0 * rightTriggerValue;
        turboFactor = limitStick(stickInput) * acceleration_factor;
        return turboFactor;
    }

    /**
     * Methods to get the value of gamepad Left stick X for Pan motion X direction.
     * This is the method to apply any directional modifiers to match to the X plane of robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.left_stick_x
     */
    public double gp1GetLeftStickX() {
        return hzGamepad1.left_stick_x;
    }
    public double gp2GetLeftStickX() {
        return hzGamepad2.left_stick_x;
    }

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Freight Frenzy Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double gp1GetLeftStickY() { return hzGamepad1.left_stick_y; }
    public double gp2GetLeftStickY() { return hzGamepad2.left_stick_y; }

    /**
     * Methods to get the value of gamepad Right stick X to keep turning.
     * This is the method to apply any directional modifiers to match to the turn direction robot.
     * No modifier needed for Hazmat Freight Frenzy Robot.
     *
     * @return gpGamepad1.right_stick_x
     */
    public double gp1GetRightStickX() {
        return hzGamepad1.right_stick_x;
    }
    public double gp2GetRightStickX() {
        return hzGamepad2.right_stick_x;
    }
    public double gp1GetRightStickY() {
        return hzGamepad1.right_stick_y;
    }
    public double gp2GetRightStickY() {
        return hzGamepad2.right_stick_y;
    }

    /**
     * Methods to get the value of gamepad Right Trigger for turbo mode (max speed).
     * This is the method to apply any modifiers to match to action of turbo mode for each driver preference.
     * For Hazmat Freight Frenzy Right Trigger pressed means turbo mode on.
     *
     * @return gpGamepad1.right_trigger
     * @return gpGamepad2.right_trigger
     */
    public double gp1GetRightTrigger() {
        return hzGamepad1.right_trigger;
    }
    public double gp2GetRightTrigger() {
        return hzGamepad2.right_trigger;
    }

    /**
     * gp1 right trigger press cubic value when pressed
     * @return
     */
    public boolean gp1GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp1RightTriggerLast && (gp1GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp1RightTriggerLast = (gp1GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * gp2 right trigger press cubic value when pressed
     * @return
     */
    public boolean gp2GetRightTriggerPress() {
        boolean isPressedRightTrigger = false;
        if (!gp2RightTriggerLast && (gp2GetRightTrigger()>0.7)) {
            isPressedRightTrigger = true;
        }
        gp2RightTriggerLast = (gp2GetRightTrigger()>0.7);
        return isPressedRightTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Trigger
     *
     * @return gpGamepad1.left_trigger
     * @return gpGamepad2.left_trigger
     */
    public double gp1GetLeftTrigger() {
        return hzGamepad1.left_trigger;
    }
    public double gp2GetLeftTrigger() {
        return hzGamepad2.left_trigger;
    }

    /**
     * The range of the gp1 left trigger cubic press
     * @return
     */
    public boolean gp1GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp1LeftTriggerLast && (gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp1LeftTriggerLast = (gp1GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * The range of the gp2 left trigger cubic press values
     * @return
     */
    public boolean gp2GetLeftTriggerPress() {
        boolean isPressedLeftTrigger = false;
        if (!gp2LeftTriggerLast && (gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        gp2LeftTriggerLast = (gp2GetLeftTrigger()>0.7);
        return isPressedLeftTrigger;
    }

    /**
     * Methods to get the value of gamepad Left Bumper
     *
     * @return gpGamepad1.left_bumper
     * @return gpGamepad2.left_bumper
     */
    public boolean gp1GetLeftBumper() {
        return hzGamepad1.left_bumper;
    }
    public boolean gp2GetLeftBumper() {
        return hzGamepad2.left_bumper;
    }

    /**
     * Method to track if Left Bumper was pressed
     * To ensure that the continuous holding of the left bumper does not cause a contiual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing hold or release of button should not trigger action.
     *
     * @return isPressedLeftBumper| = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp1LeftBumperLast && hzGamepad1.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp1LeftBumperLast = hzGamepad1.left_bumper;
        return isPressedLeftBumper;
    }

    public boolean gp2GetLeftBumperPress() {
        boolean isPressedLeftBumper = false;
        if (!gp2LeftBumperLast && hzGamepad2.left_bumper) {
            isPressedLeftBumper = true;
        }
        gp2LeftBumperLast = hzGamepad2.left_bumper;
        return isPressedLeftBumper;
    }

    /**
     * Methods to get the value of gamepad Right Bumper
     *
     * @return gpGamepad1.right_bumper
     * @return gpGamepad2.right_bumper
     */
    public boolean gp1GetRightBumper() {
        return hzGamepad1.right_bumper;
    }
    public boolean gp2GetRightBumper() {
        return hzGamepad2.right_bumper;
    }
    /**
     * Method to track if Right Bumper was pressed
     * To ensure that the continuous holding of the right bumper does not cause a continual action,
     * the state of the bumper is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedRightBumper = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp1RightBumperLast && hzGamepad1.right_bumper) {
            isPressedRightBumper = true;
        }
        gp1RightBumperLast = hzGamepad1.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp2GetRightBumperPress() {
        boolean isPressedRightBumper = false;
        if (!gp2RightBumperLast && hzGamepad2.right_bumper) {
            isPressedRightBumper = true;
        }
        gp2RightBumperLast = hzGamepad2.right_bumper;
        return isPressedRightBumper;
    }

    public boolean gp1GetRightBumperPersistant(){
        return hzGamepad1.right_bumper;
    }
    public boolean gp2GetRightBumperPersistant(){
        return hzGamepad2.right_bumper;
    }

    /**
     * Method to track if Button A was pressed
     * To ensure that the continuous holding of Button A does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButton A = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp1ButtonALast && hzGamepad1.a) {
            isPressedButtonA = true;
        }
        gp1ButtonALast = hzGamepad1.a;
        return isPressedButtonA;
    }
    public boolean gp2GetButtonAPress() {
        boolean isPressedButtonA = false;
        if (!gp2ButtonALast && hzGamepad2.a) {
            isPressedButtonA = true;
        }
        gp2ButtonALast = hzGamepad2.a;
        return isPressedButtonA;
    }
    public boolean gp1GetA(){
        return hzGamepad1.a;
    }
    public boolean gp2GetA(){
        return hzGamepad2.a;
    }


    /**
     * Method to track if Button Y was pressed
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonY = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp1ButtonYLast && hzGamepad1.y) {
            isPressedButtonY = true;
        }
        gp1ButtonYLast = hzGamepad1.y;
        return isPressedButtonY;
    }
    public boolean gp2GetButtonYPress() {
        boolean isPressedButtonY = false;
        if (!gp2ButtonYLast && hzGamepad2.y) {
            isPressedButtonY = true;
        }
        gp2ButtonYLast = hzGamepad2.y;
        return isPressedButtonY;
    }
    public boolean gp1GetY(){
        return hzGamepad1.y;
    }
    public boolean gp2GetY(){
        return hzGamepad2.y;
    }

    /**
     * Method to track if Button X was pressed
     * To ensure that the continuous holding of Button X does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonX = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp1ButtonXLast && hzGamepad1.x) {
            isPressedButtonX = true;
        }
        gp1ButtonXLast = hzGamepad1.x;
        return isPressedButtonX;
    }

    public boolean gp2GetButtonXPress() {
        boolean isPressedButtonX = false;
        if (!gp2ButtonXLast && hzGamepad2.x) {
            isPressedButtonX = true;
        }
        gp2ButtonXLast = hzGamepad2.x;
        return isPressedButtonX;
    }
    public boolean gp1GetX(){
        return hzGamepad1.x;
    }
    public boolean gp2GetX(){
        return hzGamepad2.x;
    }

    /**
     * Method to track if Button B was pressed to move Arm
     * To ensure that the continuous holding of Button Y does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedButtonB = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp1ButtonBLast && hzGamepad1.b) {
            isPressedButtonB = true;
        }
        gp1ButtonBLast = hzGamepad1.b;
        return isPressedButtonB;
    }
    public boolean gp2GetButtonBPress() {
        boolean isPressedButtonB = false;
        if (!gp2ButtonBLast && hzGamepad2.b) {
            isPressedButtonB = true;
        }
        gp2ButtonBLast = hzGamepad2.b;
        return isPressedButtonB;
    }
    public boolean gp1GetB(){
        return hzGamepad1.b;
    }
    public boolean gp2GetB(){
        return hzGamepad2.b;
    }

    /**
     * Method to track if Dpad_up was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_up = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast && hzGamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = hzGamepad1.dpad_up;
        return isPressedDpad_up;
    }
    public boolean gp2GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp2Dpad_upLast && hzGamepad2.dpad_up) {
            isPressedDpad_up = true;
        }
        gp2Dpad_upLast = hzGamepad2.dpad_up;
        return isPressedDpad_up;
    }

    public boolean gp1GetDpad_up(){
        return hzGamepad1.dpad_up;
    }
    public boolean gp2GetDpad_up(){
        return hzGamepad2.dpad_up;
    }

    /**
     * Method to track if Dpad_down was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_down = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast && hzGamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = hzGamepad1.dpad_down;
        return isPressedDpad_down;
    }
    public boolean gp2GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp2Dpad_downLast && hzGamepad2.dpad_down) {
            isPressedDpad_down = true;
        }
        gp2Dpad_downLast = hzGamepad2.dpad_down;
        return isPressedDpad_down;
    }

    public boolean gp1GetDpad_down(){
        return hzGamepad1.dpad_down;
    }
    public boolean gp2GetDpad_down(){
        return hzGamepad2.dpad_down;
    }

    /**
     * Method to track if Dpad_left was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp1Dpad_leftLast && hzGamepad1.dpad_left) {
            isPressedDpad_left = true;
        }
        gp1Dpad_leftLast = hzGamepad1.dpad_left;
        return isPressedDpad_left;
    }
    public boolean gp2GetDpad_leftPress() {
        boolean isPressedDpad_left;
        isPressedDpad_left = false;
        if (!gp2Dpad_leftLast && hzGamepad2.dpad_left) {
            isPressedDpad_left = true;
        }
        gp2Dpad_leftLast = hzGamepad2.dpad_left;
        return isPressedDpad_left;
    }

    public boolean gp1GetDpad_left(){
        return hzGamepad1.dpad_left;
    }
    public boolean gp2GetDpad_left(){
        return hzGamepad2.dpad_left;
    }

    /**
     * Method to track if Dpad_right was pressed
     * To ensure that the continuous holding of Dpad_up does not send continual triggers,
     * the state of the button is recorded and compared against previous time.
     * Only if the previous state is unpressed and current state is pressed would
     * the function return true.
     * Continue to not press, or continuing to hold or release of button should not trigger action.
     *
     * @return isPressedDpad_left = true if prev state is not pressed and current is pressed.
     */
    public boolean gp1GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp1Dpad_rightLast && hzGamepad1.dpad_right) {
            isPressedDpad_right = true;
        }
        gp1Dpad_rightLast = hzGamepad1.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp2GetDpad_rightPress() {
        boolean isPressedDpad_right;
        isPressedDpad_right = false;
        if (!gp2Dpad_rightLast && hzGamepad2.dpad_right) {
            isPressedDpad_right = true;
        }
        gp2Dpad_rightLast = hzGamepad2.dpad_right;
        return isPressedDpad_right;
    }
    public boolean gp1GetDpad_right(){
        return hzGamepad1.dpad_right;
    }
    public boolean gp2GetDpad_right(){
        return hzGamepad2.dpad_right;
    }

    public boolean gp1GetStart(){
        return hzGamepad1.start;
    }
    public boolean gp2GetStart(){
        return hzGamepad2.start;
    }

}