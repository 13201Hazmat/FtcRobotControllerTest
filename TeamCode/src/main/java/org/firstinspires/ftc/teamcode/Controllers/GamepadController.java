package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.AutoOpMode9;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;

import java.util.Objects;

/**
 * Defenition of the HzGamepad Class <BR>
 *
 * HzGamepad consists of system provided gamepad(s) and adds functionality to the selection
 * made on gamepads <BR>
 *
 * For Hazmat  PowerPlay, two Gamepads are used (gamepad1 and gamepad2) <BR>
 *
 * The controls are as follows: (replace with gamepad2 for 2nd gamepad) <BR>
 *  *      Left Stick for pan motion (gamepad1.left_stick_x and gamepad2.left_stick_y) <BR>
 *  *      Right Stick for turn motion (gamepad2.right_stick_x: gamepad1.right_stick_y) <BR>
 *  *      Right Bumper magazine flip and majorClaw state(gp2) (gamepad1.right_bumper, gamepad2.right_bumper) <BR>
 *  *      Left Bumper for spinner state and minorArm state(gp2) (gamepad1.left_bumper, gamepad2.left_bumper) <BR>
 *  *      Right Trigger for turbo, and majorArm Parking position(gp2) (gamepad1.right_trigger, gamepad2.right_trigger) <BR>
 *  *      Button A for elevator intake level and major arm pickup position(gp2) (gamepad1.a, gamepad2.a) <BR>
 *  *      Button Y for elevator level 2 and major arm capstone position(gp2) (gamepad1.y, gamepad2.y) <BR>
 *  *      Button X for elevator level 1 and majorArm down one level(gp2) (gamepad1.x, gamepad2.x) <BR>
 *  *      Button B for elevator level 3 and majorArm level up one(gp2) (gamepad1.b, gamepad2.b) <BR>
 *  *      Button Dpad_up for intake out & stop, also for minorArm level up one (gamepad1.dpad_up, gamepad2.dpad_up) <BR>
 *  *      Button Dpad_down for intake in & stop, also for minorArm level down one (gamepad1.dpad_down, gamepad2.dpad_down) <BR>
 *
 * To access the gamepad functions, use the gp1Get* or gp2Get* functions at the end of this class <BR>
 *     gp1GetLeftStickX(), gp2GetLeftStickX()
 *     gp1GetLeftStickY(), gp2GetLeftStickY()
 *     gp1GetRightStickX(), gp2GetRightStickX()
 *     gp1GetRightStickY(), gp2GetRightStickY()
 *     gp1GetLeftTrigger(), gp2GetRightTrigger()
 *     gp1GetLeftTriggerPress(), gp2GetRightTriggerPress for toggle value()
 *     gp1GetLeftBumper(), gp2GetRightBumper()
 *     gp1GetLeftBumperPress(), gp2GetRightBumperPress for toggle value()
 *     gp1GetX(), gp2GetY(), gp1GetA(), gp2GetB()
 *     gp1GetXPress(), gp2GetYPress(), gp1GetAPress(), gp2GetBPress() for toggle value()
 *     gp1GetDpad_up(), gp2GetDpad_down(). gp1GetDpad_left(), gp2GetDpad_right()
 *     gp1GetDpad_upPress(), gp2GetDpad_downPress(). gp1GetDpad_leftPress(), gp2GetDpad_rightPress()  for toggle value()
 *     gp1GetStart(), gp2GetStart()
 *
 */

public class GamepadController {

    //Create object reference to objects to systems passed from TeleOp
    public Gamepad hzGamepad1, hzGamepad2;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public Lights lights;
    public OuttakeArm outtakeArm;
    public OuttakeSlides outtakeSlides;

    /**
     * Constructor for HzGamepad1 and HzGamepad2 class that extends gamepad.
     * Assign the gamepad1 and gamepad2 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad hzGamepad1,
                             Gamepad hzGamepad2,
                             DriveTrain driveTrain,
                             IntakeArm intakeArm,
                             IntakeSlides intakeSlides,
                             OuttakeArm outtakeArm,
                             OuttakeSlides outtakeSlides,
                             Lights lights
                            ) {
        this.hzGamepad1 = hzGamepad1;
        this.hzGamepad2 = hzGamepad2;
        this.driveTrain = driveTrain;
        this.intakeArm = intakeArm;
        this.intakeSlides = intakeSlides;
        this.outtakeArm = outtakeArm;
        this.outtakeSlides = outtakeSlides;
        this.lights = lights;
    }



    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runIntakeArm();
        runIntakeSlides();
        runOuttakeArm();
        runOuttakeSlides();
        //recordAndReplay();
        runDriveControl_byRRDriveModes();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     */
    // RR Drive Train
    public void runDriveControl_byRRDriveModes() {
/*
        //driveTrain.poseEstimate = driveTrain.getPoseEstimate();

        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;

        if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
            driveTrain.gamepadInput = new Vector2d(
                    -gp1TurboMode(gp1GetLeftStickY()),
                    -gp1TurboMode(gp1GetLeftStickX()));
        }
        driveTrain.gamepadInputTurn = gp1TurboMode(-gp1GetRightStickX());

        driveTrain.driveTrainPointFieldModes();

        /*if (driveTrain.driveType == DriveTrain.DriveType.FIELD_CENTRIC){

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
            driveTrain.driveTrainFieldCentric();
        }*/
    }

    public void runIntakeSlides(){
        if (!gp1GetStart() && gp1GetB()) {
            intakeSlides.modifyIntakeSlidesLength(0.33 * (1.0 + 1.0 * gp1GetLeftTrigger()));
            if (intakeArm.intakeArmState == IntakeArm.INTAKE_ARM_STATE.INIT) {
                intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1);
            }
        } else if (gp1GetX()) {
            intakeSlides.modifyIntakeSlidesLength(-0.33 * (1.0 + 1.0 * gp1GetLeftTrigger()));
        } else {
            intakeSlides.modifyIntakeSlidesLength(0);
        }
        if(gp1GetStart() && gp1GetX()){
            intakeSlides.moveIntakeSlidesToMinRetracted();
        }
    }

    ElapsedTime tempAutoCloseDisableTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean tempAutoCloseDisableFlag = false;

    public void runIntakeArm(){
        if (gp1GetStart() && gp1GetRightBumperPress()) {
            intakeArm.autoIntakeCloseMode = !intakeArm.autoIntakeCloseMode;
        }

        if (tempAutoCloseDisableFlag && tempAutoCloseDisableTimer.time() > 3000) {
            tempAutoCloseDisableFlag = false;
        }

        if (intakeArm.autoIntakeCloseMode && intakeArm.senseIntakeCone() &&
                outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.OPEN
                && !tempAutoCloseDisableFlag
                /*&& !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.PICKUP_FALLEN_CONE)*/) {
            intakeArm.closeGrip();
        }

        if (gp1GetRightBumperPress() && intakeArm.intakeArmState != IntakeArm.INTAKE_ARM_STATE.TRANSFER &&
                intakeArm.intakeArmState != IntakeArm.INTAKE_ARM_STATE.INIT) {
            if(intakeArm.intakeGripState == IntakeArm.INTAKE_GRIP_STATE.CLOSED){
                intakeArm.openGrip();
                if (gp1GetDpad_left()) {
                    tempAutoCloseDisableTimer.reset();
                    tempAutoCloseDisableFlag = true;
                }
            } else if(outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.OPEN){
                intakeArm.closeGrip();
                if(intakeArm.intakeArmState == IntakeArm.INTAKE_ARM_STATE.PICKUP_FALLEN_CONE){
                    intakeArm.moveWrist(IntakeArm.INTAKE_ARM_STATE.AUTO_CONE_5);
                    intakeArm.moveWristUp();
                }
            }
        }

        if (!gp1GetStart() && gp1GetDpad_upPress()){
            if (intakeArm.intakeArmState == IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION) {
                if (isOuttakeAtTransfer()) {
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
                }
            } else {
                intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION);
            }
        }

        if(!gp1GetStart() && gp1GetDpad_upPress() && intakeArm.intakeArmState == IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION){
            intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
        }

        if (gp1GetStart() && gp1GetDpad_up()){
            intakeArm.continousArmRotateUp();
        }

        if (!gp1GetStart() && gp1GetDpad_downPress()) {
            intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1);
            intakeArm.openGrip();
        }

        if (gp1GetStart() && gp1GetDpad_down()) {
            intakeArm.continousArmRotateDown();
        }

        if (!gp1GetStart() && gp1GetDpad_rightPress()) {
            intakeArm.moveWristUp();
        }

        if (gp1GetStart() && gp1GetDpad_rightPress()) {
            intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.PICKUP_FALLEN_CONE);
            intakeArm.openGrip();
        }

        if (gp1GetButtonYPress()) {
            //intakeArm.moveArmWristUpOneStack();

            if (intakeArm.intakeArmState.index < 1 || intakeArm.intakeArmState.index > 7) {
                return;
            } else {
                if (intakeArm.intakeArmState.index + 1 == 8) {
                    if (isOuttakeAtTransfer()) {
                        assert intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index + 1) != null;
                        intakeArm.moveArm(intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index + 1));
                    }
                } else {
                    assert intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index + 1) != null;
                    intakeArm.moveArm(intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index + 1));
                }
            }
        }

        if (!gp1GetStart() && gp1GetButtonAPress()) {
            if (intakeArm.intakeArmState.index <= 1 || intakeArm.intakeArmState.index > 8) {
                return;
            } else {
                assert intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index - 1) != null;
                intakeArm.moveArm(intakeArm.intakeArmState.byIndex(intakeArm.intakeArmState.index - 1));
            }
        }

        if (gp1GetLeftBumperPress()
                && intakeArm.intakeGripState == IntakeArm.INTAKE_GRIP_STATE.CLOSED
                && intakeArm.senseIntakeCone()) {
            runTransferSequence();
        }

    }

    public void runOuttakeSlides(){
        if(!gp2GetLeftBumper() && outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.CLOSED && gp2GetButtonXPress()){
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.LOW_JUNCTION);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.LOW_JUNCTION);
        }

        if(gp2GetLeftBumper() && gp2GetButtonXPress()) {
            outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.TELEOP_RIGHT);
        }

        if(outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.CLOSED && gp2GetButtonYPress() ){
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.MEDIUM_JUNCTION);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
        }

        if(!gp2GetStart() && !gp2GetLeftBumper() &&
                outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.CLOSED && gp2GetButtonBPress()){
            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.HIGH_JUNCTION);
            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
        }

        if(!gp2GetStart() && gp2GetLeftBumper() && gp2GetButtonBPress()){
            outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.TELEOP_LEFT);
        }

        if(!gp2GetStart() && gp2GetButtonAPress()){
            outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.CENTER);
        }

        if(gp2GetLeftStickY()>0.05|| gp2GetLeftStickY()<-0.05) {
            outtakeSlides.modifyOuttakeSlidesLength(gp2TurboMode(-gp2GetLeftStickY()));
        }

        outtakeSlides.moveTurretDelta(-gp2GetRightStickX() * 0.5 *(1.0 + 1.0 * gp2GetRightTrigger())); // Linear

        //TODO : Test with no color sensor
        if (gp2GetStart() && (-gp2GetLeftStickY() < -0.5)) {
            outtakeSlides.manualResetOuttakeMotor();
        }

    }

    public boolean isOuttakeAtTransfer(){
         return (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)
                 && outtakeArm.isOuttakeWristInState(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER)
                 && outtakeArm.isOuttakeGripInState(OuttakeArm.OUTTAKE_GRIP_STATE.OPEN)
                 && outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER));
    }

    public boolean moveOuttakeToTransferFlag = false;
    public ElapsedTime outtakeGripTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void runOuttakeArm(){
        if (outtakeArm.outtakeWristState != OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER ) {
            if (gp2GetRightBumperPress()) {
                if (outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.CLOSED &&
                        (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP) ||
                                outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.LOW_JUNCTION))) {
                    outtakeArm.openGrip();
                    safeWait(300);
                    moveOuttakeToTransfer();
                } else {
                    moveOuttakeToTransfer();
                }
            }
        }

        if (outtakeArm.runWristInDropFlag) {
            if (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP)) {
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_DROP);
                outtakeArm.runWristInDropFlag = false;
            } else if (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.LOW_JUNCTION)) {
                outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_LOW_JUNCTION);
                outtakeArm.runWristInDropFlag = false;
            }
        }

        if (outtakeArm.outtakeArmState != OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER
                && gp2GetDpad_upPress()) {
            outtakeArm.moveOuttakeWristUp();
        }

        if (outtakeArm.outtakeArmState != OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER
                && gp2GetDpad_downPress()) {
            outtakeArm.moveOuttakeWristDown();
        }

        /*if (outtakeArm.outtakeArmState == OuttakeArm.OUTTAKE_ARM_STATE.DROP) {
            if (outtakeArm.senseJunction()) {
                lights.setPattern(Lights.REV_BLINKIN_PATTERN.OUTTAKE_JUNCTION_ALIGNED);
            } else {
                lights.setPattern(Lights.REV_BLINKIN_PATTERN.OUTTAKE_JUNCTION_NOT_ALIGNED);
            }
        }*/

        if (outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.OPEN) {
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEFAULT);
        }
    }

    public void moveOuttakeToTransfer(){
        //outtakeArm.closeGrip();
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.openGrip();
    }

    public ElapsedTime transferCycleTime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public double transferCycleTimeVal = 0;
    boolean intakeArmNotAtPickup = false;
    public void runTransferSequence(){
        transferCycleTime.reset();
        if (intakeArm.intakeArmState != IntakeArm.INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1){
            intakeArmNotAtPickup = true;
        }
        lights.setPattern(Lights.REV_BLINKIN_PATTERN.TRANSFER_PROGRESS);
        ElapsedTime transferTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        ElapsedTime stackTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
        if(intakeArmNotAtPickup){
            safeWait(300);
        }
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        intakeSlides.runIntakeMotorToLevel();
        if(!isOuttakeAtTransfer()){
            moveOuttakeToTransfer();
        }
        transferTimer.reset();
        while(transferTimer.time() < 200 /*2000*/&& (!isOuttakeAtTransfer() || //500
                !intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER))){
            runDriveControl_byRRDriveModes();
        }
        if (isOuttakeAtTransfer()) {
            intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
            transferTimer.reset();
            while( !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER) && transferTimer.time() < 500){ //1000
                runDriveControl_byRRDriveModes();
            }
        } else {
            //Outtake has some error
            intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION);
            return;
        }
        transferTimer.reset();
        //safeWait(400); // Commented out Feb 13
        while(!outtakeArm.senseOuttakeCone()  && transferTimer.time() < 300){//1500
            runDriveControl_byRRDriveModes();
        }
        intakeArm.openGrip();
        safeWait(100); //200
        outtakeArm.closeGrip();
        safeWait(100); //100

        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
        transferTimer.reset();
        while (transferTimer.time() < 500 && !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.INIT)){ //700
            runIntakeSlides();
            runDriveControl_byRRDriveModes();
        }
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.MEDIUM_JUNCTION);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
        /*transferTimer.reset(); // Set up delay for goBilda
        while(transferTimer.time() < 300){ //500 // Allow Intake controls
            runOuttakeSlides();
            runIntakeSlides();
            runIntakeArm();
            runDriveControl_byRRDriveModes();
        }*/
        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_DROP);
        safeWait(200); //500
        lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);
        transferCycleTimeVal = transferCycleTime.time();
    }

    public void safeWait(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (timer.time() < time) {
            runDriveControl_byRRDriveModes();
        }
    }

    /* Auto In TeleOp

    public enum INTAKE_STATE{
        I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, IExit;
    }
    public INTAKE_STATE intakeState = INTAKE_STATE.I0;

    public enum OUTTAKE_STATE{
        O0, O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11;
    }
    public OUTTAKE_STATE outtakeState = OUTTAKE_STATE.O0;

    public void teleOpAutoPickAndDropStateMachine(){
        //telemetry.setAutoClear(true);
        cycleTimer.reset();
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        while(opModeIsActive() && !isStopRequested() &&
                dropConeCounter < dropConeCount && !timeoutExit) {
            stateMachineLoopCounter ++;
            switch (outtakeState) {
                case O0: // Start of State machine
                    outtakeState = AutoOpMode9.OUTTAKE_STATE.O1;
                    break;

                case O1: // Cone is sensed, close grip
                    if (dropConeCounter == 0 && !intakeGripClosed) {
                        outtakeArm.closeGrip();
                        outtakeGripClosed = true;
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O3;
                    } else if (intakeState == AutoOpMode9.INTAKE_STATE.I13) {
                        outtakeArm.closeGrip();
                        outtakeGripClosed = true;
                        outtakeGripTimer.reset();
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O2;
                    }
                    break;

                case O2: // If Intake Arm is not in transfer, proceed to move outttake to drop position
                    if((intakeState == AutoOpMode9.INTAKE_STATE.I1 || intakeState == AutoOpMode9.INTAKE_STATE.I2 || intakeState == AutoOpMode9.INTAKE_STATE.I3)
                            && outtakeGripTimer.time() > 200 &&
                            !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)) {
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O3;
                    }
                    break;

                case O3: // Move outtake to drop Position
                    outtakeArm.moveOuttakeGuide(OuttakeArm.OUTTAKE_GUIDE_STATE.UP);
                    outtakeSlides.moveOuttakeSlides(outtakeSlidesDropState);
                    outtakeArm.moveArm(outtakeArmDropState);
                    outtakeWristTimer.reset();
                    outtakeState = AutoOpMode9.OUTTAKE_STATE.O4;
                    break;

                case O4: // Move outtake wrist to Drop
                    //if (outtakeWristTimer.time() > 0) { //300 for Gobilda servo
                    outtakeArm.moveWrist(outtakeWristDropState);
                    //outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_AUTO_HIGH_JUNCTION);
                    outtakeState = AutoOpMode9.OUTTAKE_STATE.O5;
                    outtakeWristTimer.reset();
                    //}
                    break;

                case O5: // Verify that Outtake slide, Arm and Wrist are in position to drop
                    if (dropConePosition == AutoOpMode9.DROP_CONE_POSITION.MEDIUM) {
                        safeWait(500); //1000
                    }
                    if ((outtakeWristTimer.time() > 500 && //TODO :
                            outtakeSlides.isOuttakeSlidesInState(outtakeSlidesDropState)
                            && outtakeArm.isOuttakeArmInState(outtakeArmDropState)
                            && outtakeArm.isOuttakeWristInState(outtakeWristDropState))
                            || outtakeWristTimer.time() > 1500) {//1000
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O6;
                    }
                    break;

                case O6: // Open Grip and Drop Cone
                    outtakeArm.openGrip();
                    outtakeGripTimer.reset();
                    outtakeState = AutoOpMode9.OUTTAKE_STATE.O7;
                    break;

                case O7: // Wait for cone to be dropped completely
                    if (outtakeGripTimer.time() > 200) {
                        outtakeGripClosed = false;
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O8;
                    }
                    break;

                case O8: // Check if intake is in Transfer, else Move outtake to transfer
                    if (!intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)) {
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O9;
                    }
                    break;

                case O9: // Move outtake to Transfer
                    outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                    outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
                    outtakeGripTimer.reset();
                    switch (dropConeCounter) {
                        case 0:
                            timeToDropPreloadCone = gameTimer.time();
                            break;
                        case 1:
                            timeToDropCone1 = gameTimer.time();
                            break;
                        case 2:
                            timeToDropCone2 = gameTimer.time();
                            break;
                        case 3:
                            timeToDropCone3 = gameTimer.time();
                            break;
                        case 4:
                            timeToDropCone4 = gameTimer.time();
                            break;
                        case 5:
                            timeToDropCone5 = gameTimer.time();
                            break;
                    }
                    dropConeCounter++;
                    outtakeState = AutoOpMode9.OUTTAKE_STATE.O10;

                case O10: // Check if outtake is at transfer
                    //telemetry.addData("outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)", outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER));
                    //telemetry.addData("outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)", outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER));
                    if((outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)
                            && outtakeArm.isOuttakeWristInState(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER)
                            && outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.OPEN
                            && outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER))
                            || outtakeGripTimer.time()>300) { //650
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O11;
                    }
                    break;

                case O11: // Wait till Intake drops cone
                    //telemetry.addData("outtakeArm.senseOuttakeCone()", outtakeArm.senseOuttakeCone());
                    if (dropConeCounter < dropConeCount) {
                        if (intakeState == AutoOpMode9.INTAKE_STATE.I12) {
                            if (outtakeArm.senseOuttakeCone() || outtakeSenseTimer.time() > 500) { //500
                                outtakeState = AutoOpMode9.OUTTAKE_STATE.O1;
                                cycleTime = cycleTimer.time();
                                cumulativeCycleTime +=cycleTime;
                                cycleTimer.reset();
                            }
                        }
                    } else {
                        outtakeState = AutoOpMode9.OUTTAKE_STATE.O1;
                        cycleTime = cycleTimer.time();
                        cumulativeCycleTime +=cycleTime;
                        cycleTimer.reset();
                    }
                    break;
            }

            switch (intakeState) {
                case I0: // Start of State Machine
                    intakeState = AutoOpMode9.INTAKE_STATE.I1;
                    break;

                case I1: // Start fo next round of intake sequence, if stack is not empty
                    // if only drop preloaded, Intake state machine wont move forward
                    if(stackConeCounter < stackConeCount) {
                        intakeState = AutoOpMode9.INTAKE_STATE.I2;
                    }
                    break;

                case I2: //Move Intake arm and slides to stack current cone level
                    intakeArm.moveArm(Objects.requireNonNull(intakeArm.intakeArmState.byIndex(5 - stackConeCounter)));
                    if (5-stackConeCounter <=2) {
                        safeWait(300);
                    }
                    intakeSlides.moveIntakeSlides(Objects.requireNonNull(intakeSlides.intakeSlidesState.byIndex(5 - stackConeCounter)));
                    intakeSlides.runIntakeMotorToLevel();
                    intakeGripTimer.reset();
                    intakeState = AutoOpMode9.INTAKE_STATE.I3;
                    break;

                case I3: // Wait for Outtake grip to be open, and hold position minimum for 500 to stabilize
                    if( (intakeSlides.isIntakeSlidesInState(intakeSlides.intakeSlidesState.byIndex(5 - stackConeCounter))
                            && outtakeArm.isOuttakeGripInState(OuttakeArm.OUTTAKE_GRIP_STATE.OPEN)
                            && !outtakeGripClosed
                            && intakeGripTimer.time() > 300)
                            || intakeGripTimer.time() > 1000) {//400
                        intakeState = AutoOpMode9.INTAKE_STATE.I4;
                    }
                    break;

                case I4: // Close grip on stack
                    if (outtakeArm.isOuttakeGripInState(OuttakeArm.OUTTAKE_GRIP_STATE.OPEN)
                            && !outtakeGripClosed) {
                        intakeArm.closeGrip();
                        intakeGripClosed = true;
                        intakeGripTimer.reset();
                        intakeState = AutoOpMode9.INTAKE_STATE.I5;
                    }
                    break;

                case I5 : // Wait for grip to be closed completely
                    if (intakeGripTimer.time() > 400 && intakeArm.isIntakeGripInState(IntakeArm.INTAKE_GRIP_STATE.CLOSED)) {
                        intakeState = AutoOpMode9.INTAKE_STATE.I6;
                    }
                    break;

                case I6: // Move Arm up to clear stack
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION);
                    intakeArm.moveIntakeWristToTransfer();
                    intakeArmTimer.reset();
                    intakeState = AutoOpMode9.INTAKE_STATE.I7;
                    break;

                case I7: // Wait for Arm to be completely moved up of stack
                    if (intakeArmTimer.time() > 300) {
                        intakeState = AutoOpMode9.INTAKE_STATE.I8;
                    }
                    break;
                case I8: // Move intake slides to Transfer
                    intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
                    intakeArm.moveIntakeWristToTransfer();
                    intakeState = AutoOpMode9.INTAKE_STATE.I9;
                    if (gameTimer.time() >27000) { //27000
                        timeoutExit = true;
                    }
                    break;

                case I9: // Wait till Outtake is ready to accept cone
                    if(outtakeState == AutoOpMode9.OUTTAKE_STATE.O11){
                        intakeState = AutoOpMode9.INTAKE_STATE.I10;
                    }
                    break;

                case I10: // Move intake Arm to transfer
                    intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
                    intakeArm.moveIntakeWristToTransfer();
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
                    intakeArmTimer.reset();
                    intakeState = AutoOpMode9.INTAKE_STATE.I11;
                    break;

                case I11: // Check if intake Arm and Slides are in Transfer
                    //telemetry.addData("intakeArm.isIntakeArmInTransfer", intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER) );
                    //telemetry.addData("intakeSlides.isIntakeSlidesInTransfer", intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER));

                    if((intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)
                            && intakeArm.isIntakeWristInState(IntakeArm.INTAKE_WRIST_STATE.TRANSFER)
                            && intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER))
                            || intakeArmTimer.time() > 650) { //850
                        outtakeSenseTimer.reset();
                        intakeState = AutoOpMode9.INTAKE_STATE.I12;
                    }
                    break;

                case I12: // Open Intake Grip to drop cone to Transfer
                    if (outtakeState == AutoOpMode9.OUTTAKE_STATE.O1) {
                        intakeArm.openGrip();
                        safeWait(300);
                        intakeGripClosed = false;
                        intakeState = AutoOpMode9.INTAKE_STATE.I13;
                    }
                    break;
                case I13:
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
                    intakeArmTimer.reset();
                    while (intakeArmTimer.time() < 700 &&
                            !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.INIT)) {}
                    stackConeCounter++;
                    intakeState = AutoOpMode9.INTAKE_STATE.I1;
            }
            telemetry.addData("dropConeCounter", dropConeCounter);
            telemetry.addData("stackConeCounter", stackConeCounter);
            telemetry.addData("cycleTime", cycleTime);
            telemetry.addData(" --- OuttakeState", outtakeState);
            telemetry.addData(" --- IntakeState", intakeState);
            //telemetry.addData("Stack Position", intakeSlides.intakeSlidesState.byIndex(5 - (stackConeCounter-1)));
            //telemetry.addData("Stack Position", intakeSlides.intakeSlidesState.byIndex(5 - (stackConeCounter-1)).motorPosition);
            //telemetry.addData("Intake Slide Left Motor", intakeSlides.intakeMotorLeft.getCurrentPosition());
            //telemetry.addData("Intake Slide Right Motor", intakeSlides.intakeMotorLeft.getCurrentPosition());
            //telemetry.addData("Intake touch Pressed", intakeSlides.intakeTouch.isPressed());
            telemetry.addData("Outtake Slides", outtakeSlides.outtakeMotor.getCurrentPosition());

            telemetry.addData("Pose Estimate", driveTrain.getPoseEstimate());

            telemetry.update();
            if (startPosition == AutoOpMode9.START_POSITION.TEST_POSE) {
                safeWait(0);
            }
            //safeWait(1000);

        }
        autoCorrectPosition.exit();
    }*/







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
    public double gp2TurboMode(double stickInput) {

        double acceleration_factor;
        double rightTriggerValue;

        double turboFactor;

        rightTriggerValue = gp2GetRightTrigger();
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

    public boolean gp1GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp1GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
        return isPressedLeftTrigger;
    }

    public boolean gp2GetLeftTriggerPersistent() {
        boolean isPressedLeftTrigger = false;
        if ((gp2GetLeftTrigger()>0.7)) {
            isPressedLeftTrigger = true;
        }
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