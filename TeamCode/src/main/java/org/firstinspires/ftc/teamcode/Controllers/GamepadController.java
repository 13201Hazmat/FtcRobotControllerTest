package org.firstinspires.ftc.teamcode.Controllers;

import static android.os.SystemClock.sleep;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AadiGeometry.AadiPose;
import org.firstinspires.ftc.teamcode.AadiGeometry.AadiVector;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.SystemState;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

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
    public Arm arm;
    public Hand hand;
    public Lights lights;
    public Shoulder shoulder;
    public Turret turret;

    /**
     * Constructor for HzGamepad1 and HzGamepad2 class that extends gamepad.
     * Assign the gamepad1 and gamepad2 given in OpMode to the gamepad used here.
     */
    public GamepadController(Gamepad hzGamepad1,
                             Gamepad hzGamepad2,
                             DriveTrain driveTrain,
                             Arm arm,
                             Hand hand,
                             Shoulder shoulder,
                             Turret turret,
                             Lights lights
                            ) {
        this.hzGamepad1 = hzGamepad1;
        this.hzGamepad2 = hzGamepad2;
        this.driveTrain = driveTrain;
        this.arm = arm;
        this.hand = hand;
        this.shoulder = shoulder;
        this.turret = turret;
        this.lights = lights;
    }



    /**
     *runByGamepad is the main controller function that runs each subsystem controller based on states
     */
    public void runByGamepadControl(){
        runDriveControl_byRRDriveModes();
        runTurret();
        runShoulderArmCombo();
        runRecordAndReplay();
        runHand();
    }

    /**
     * runByGamepadRRDriveModes sets modes for Road Runner such as ROBOT and FIELD Centric Modes. <BR>
     */
    // RR Drive Train
    public void runDriveControl_byRRDriveModes() {

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

    /**
     * runArm sets the differnt intake controls, if intake should take in freight(Dpad_downPress) or the intake should run the opposite
     * direction in order for a stuck freight to be out of intake. <BR>
     */
    public void runTurret(){

        //Move turret based on Right Stick on Gamepad 2
        if (gp2GetRightStickX() >= 0.2)  {
            turret.rotateTurret(Math.pow(gp2GetRightStickX() * 1.25 - 0.25, 3));
        } else if (gp2GetRightStickX() <= -0.2) {
            turret.rotateTurret(Math.pow(gp2GetRightStickX() * 1.25 + 0.25, 3));
        }

        //Move turret to face forward
        if (gp1GetButtonYPress()) {
            turret.faceForward();
        }

        //Move turret to face right
        if (gp1GetButtonBPress()) {
            turret.faceRight();
        }

        //Move turret to face left
        if (gp1GetButtonXPress()) {
            turret.faceLeft();
        }

        //manual reset for the turret
        if (gp2GetStart()){
            if (gp2GetRightStickX() != 0){
                turret.manualResetTurret(gp2GetRightStickX());
            }
        }

        if (turret.runTurretToLevelState) {
            arm.convertMotorEncoderValueToArmLength();
            //Lowers the power based on the extension length of the arm
            turret.runTurretToPosition(turret.TURRET_POWER
                    - (SystemState.ArmExtension * SystemState.ARM_EXTENSION_POWER_MULTIPLIER));
        }
        SystemState.TurretState = turret.turretMotorState;
    }

    public boolean comboMotion = false;
    /**
     * runArm sets the differnt intake controls, if intake should take in freight(Dpad_downPress) or the intake should run the opposite
     * direction in order for a stuck freight to be out of intake. <BR>
     */
    public void runShoulderArmCombo(){

        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);


        //Move arm to low junction if Gamepad 2 X is pressed
        if (gp2GetButtonXPress()){
            comboMotion = true;
            arm.moveArmToLowJunction();
            shoulder.moveToShoulderLowJunction();
            hand.moveWristLevel(shoulder.shoulderCurrentPosition);
        }

        //Moves arm to the high junction if Gamepad 2 B is pressed
        if (!gp2GetStart() && gp2GetButtonBPress()){
            comboMotion = true;
            arm.moveArmToHighJunction();
            shoulder.moveToShoulderHighJunction();
        }

        //Moves arm to pickup / ground junction if Gamepad 2 A is pressed
        if (gp2GetButtonAPress()){
            comboMotion = true;
            arm.moveArmToPickUp();
            shoulder.moveShoulderToPickup();
        }

        //Moves arm to middle junction if Gamepad 2 Y is pressed
        if (gp2GetButtonYPress()){
            comboMotion = true;
            arm.moveArmToMediumJunction();
            shoulder.moveToShoulderMediumJunction();
        }

        // To protect Arm from hitting the ground
        double shoulderToArmFactor = ((shoulder.THRESHOLD_POSITION - shoulder.PICKUP_POSITION)/
                ((arm.MAX_EXTENDED_POSITION - arm.PICKUP_POSITION)));

        //Extend the shoulder based on the Gamepad 2 left and right trigger
        if (gp2GetRightTrigger() > 0.2) {
            shoulder.raiseShoulder(Math.pow(gp2GetRightTrigger() * 1.25 - 0.25, 3));
        } else if(gp2GetLeftTrigger() > 0.2) { //retract the arm based on the right joystick
            shoulder.lowerShoulder(Math.pow(gp2GetLeftTrigger() * 1.25 - 0.25, 3));
            // TODO: if shoulder is pulled below THRESHOLD, change the max length of arm such that it does not touch the ground
            /*if (shoulder.leftShoulderMotor.getCurrentPosition() < shoulder.THRESHOLD_POSITION) {
                arm.dynamicMaxExtendedPosition = (int) (shoulder.leftShoulderMotor.getCurrentPosition()/shoulderToArmFactor);
                if (arm.armMotor.getCurrentPosition() > arm.dynamicMaxExtendedPosition) {
                    arm.moveArmToDynamicMaxExtended();
                }
            }*/
        }

        //Move arm based on Left Stick on Gamepad 2
        if (gp2GetLeftStickY() >= 0.2) {//Extend Arm, since left_Stick_y is negative when pushed forward
            arm.modifyArmLength(Math.pow(-gp2GetLeftStickY() * 1.25 - 0.25, 3));
            //TODO: if arm is extended when shoulder is below threshold, change the shoulder min position to make sure shoulder is pulled up  to keep the arm below ground
            /*if (shoulder.leftShoulderMotor.getCurrentPosition() < shoulder.THRESHOLD_POSITION) {
                if (arm.armMotor.getCurrentPosition() > arm.PICKUP_POSITION) {
                    shoulder.dynamicMinPosition = (int) arm.armMotor.getCurrentPosition() * shoulderToArmFactor;
                    if (shoulder.leftShoulderMotor.getCurrentPosition() < shoulder.dynamicMinPosition) {
                        shoulder.moveShoulderToDynamicMinExtended();
                    }
                }
            }*/
        } else if (gp2GetLeftStickY() <= -0.2) { // Retract arm
            arm.modifyArmLength(Math.pow(-gp2GetLeftStickY() * 1.25 + 0.25, 3));
        }

        // Run Arm motor if position is changed
        if (arm.runArmToLevelState) {

            if (arm.armMovementDirection == Arm.ARM_MOVEMENT_DIRECTION.EXTEND) {
                arm.runArmToLevel(arm.ARM_POWER_EXTEND);
            } else {
                arm.runArmToLevel(arm.ARM_POWER_RETRACT);
            }
            if (comboMotion && (shoulder.shoulderMovementDirection == Shoulder.SHOULDER_MOVEMENT_DIRECTION.DOWN)) {
                timer.reset();
                while (timer.time() < 3000){
                    runDriveControl_byRRDriveModes();
                    runTurret();
                };
            }
        }
        SystemState.ArmState = arm.armMotorState;

        /*if (shoulder.leftShoulderMotor.getCurrentPosition() < shoulder.dynamicMinPosition) {
            shoulder.moveShoulderToDynamicMinExtended();
        }*/


        //Run Shoulder motors if position is changed
        if (shoulder.runShoulderToLevelState){
            if (shoulder.shoulderMovementDirection == Shoulder.SHOULDER_MOVEMENT_DIRECTION.UP) {
                shoulder.runShoulderToLevel(shoulder.SHOULDER_POWER_UP);
            } else {
                shoulder.runShoulderToLevel(shoulder.SHOULDER_POWER_DOWN);
            }
        }
        SystemState.ShoulderState = shoulder.shoulderState;

        //manual reset for the arm
        if (gp2GetStart()) {
            if (gp2GetLeftStickY() < 0) {
                arm.manualResetArm();
            }
        }

        //manual reset for shoulder
        if (gp2GetStart()){
            if (gp2GetLeftTrigger() > 0){ //manual reset for shoulder
                shoulder.manualResetShoulder();
            }
        }
    }

    public void runRecordAndReplay(){
        //TODO
        // gamepad1.lefttrigger + X Y A B = Record AadiPose
        //gamepad1.leftbumper + X Y A B = Replay AadiPose
    }

    //Move subsysetms to a specific AadiPose
    public void moveToAadiPose(AadiPose aadiPose){
        arm.moveArmToLength(aadiPose.getArmLength(aadiPose));
        shoulder.moveShoulderToAngle(aadiPose.getShoulderAngle(aadiPose));
        if (aadiPose.getWristAngle(aadiPose) == AadiVector.WRIST_ANGLE.LEVEL) {
            hand.moveWristLevel(aadiPose.getShoulderAngle(aadiPose));
        } else {
            hand.moveWristUp(aadiPose.getShoulderAngle(aadiPose));
        }
        turret.moveTurretToAngle(aadiPose.getTurretAngle(aadiPose));
    }


    /**
     * runArm sets the differnt intake controls, if intake should take in freight(Dpad_downPress) or the intake should run the opposite
     * direction in order for a stuck freight to be out of intake. <BR>
     */
    public void runHand(){
        if (gp2GetDpad_upPress() || gp1GetDpad_upPress()) {
            if (hand.wristState == Hand.WRIST_STATE.WRIST_DOWN) {
                //hand.moveWristLevel(shoulder.shoulderCurrentPosition);
                hand.wristState = Hand.WRIST_STATE.WRIST_LEVEL;

            } else if (hand.wristState == Hand.WRIST_STATE.WRIST_LEVEL) {
                //hand.moveWristUp(shoulder.shoulderCurrentPosition);
                hand.wristState = Hand.WRIST_STATE.WRIST_UP;

            }
        }

        if (gp2GetDpad_downPress() || gp1GetDpad_downPress()) {
            if (hand.wristState == Hand.WRIST_STATE.WRIST_UP || hand.wristState == Hand.WRIST_STATE.WRIST_UP_MAX) {
                //hand.moveWristLevel(shoulder.shoulderCurrentPosition);
                hand.wristState = Hand.WRIST_STATE.WRIST_LEVEL;
            } else if ((hand.wristState == Hand.WRIST_STATE.WRIST_LEVEL)
                && (shoulder.shoulderState == Shoulder.SHOULDER_STATE.PICKUP)) {
                arm.moveArmToPickUpWristDown();
                shoulder.moveShoulderToPickUpWristDown();

                //hand.moveWristDown();
                /*TODO : DONT UNCOMMENT THIS TILL PROTECTION IS SET FOR
                 ARM TO BE HIGH WHEN WRIST IS POINTING DOWN OR IT WILL BREAK THE WRIST*/
            }
        }

        if (hand.wristState == Hand.WRIST_STATE.WRIST_LEVEL) {
            hand.moveWristLevel(shoulder.shoulderCurrentPosition);
        }//works

        if (hand.wristState == Hand.WRIST_STATE.WRIST_UP) {
            hand.moveWristUp(shoulder.shoulderCurrentPosition);
        }//works

        if (gp2GetRightBumperPress() || gp1GetRightBumperPress()) {
            hand.toggleGrip();
        }//works

        SystemState.HandGripState = hand.gripState;
        SystemState.HandWristState = hand.wristState;
    }

    public void runResetArmShoulderTurretHand(){
    // TODO
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