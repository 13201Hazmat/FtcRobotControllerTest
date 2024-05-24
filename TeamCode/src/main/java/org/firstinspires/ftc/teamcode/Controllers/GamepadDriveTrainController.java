package org.firstinspires.ftc.teamcode.Controllers;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class GamepadDriveTrainController extends Thread{

    Gamepad hzGamepad1;
    DriveTrain driveTrain;
    LinearOpMode teleOpMode;


    public GamepadDriveTrainController(Gamepad hzGamepad1, DriveTrain driveTrain, LinearOpMode teleOpMode) {
        this.hzGamepad1 = hzGamepad1;
        this.driveTrain = driveTrain;
        this.teleOpMode = teleOpMode;
    }

    public void run(){
        while (teleOpMode.opModeIsActive() && !teleOpMode.isStopRequested() && !exit) {
            runDriveControl_byRRDriveModes();
        }
    }

    boolean exit = false;
    public void exit(){
        exit = true;
    };

    public void runDriveControl_byRRDriveModes() {

        driveTrain.gamepadInputTurn = gp1TurboMode(-gp1GetRightStickX());
        //driveTrain.gamepadInputTurn = gp1TurboMode(-gp1GetLeftStickX());

        if (driveTrain.driveType == DriveTrain.DriveType.ROBOT_CENTRIC){
            driveTrain.gamepadInput = new Vector2d(
                    -gp1TurboMode(gp1GetLeftStickY()),
                    -gp1TurboMode(gp1GetLeftStickX()));
                    //-gp1TurboMode(gp1GetRightStickY()),
                    //-gp1TurboMode(gp1GetRightStickX()));
        }

        if (driveTrain.driveType == DriveTrain.DriveType.FIELD_CENTRIC){

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) { // Red Alliance
                driveTrain.gamepadInput = driveTrain.rotateFieldCentric(
                        gp1TurboMode(gp1GetLeftStickY()),
                        gp1TurboMode(gp1GetLeftStickX()),
                        -driveTrain.pose.heading.log()
                );
                driveTrain.gamepadInput = driveTrain.pose.heading.inverse().times(
                        new Vector2d(-driveTrain.gamepadInput.x, driveTrain.gamepadInput.y));
            };

            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) { // Blue Alliance
                driveTrain.gamepadInput = driveTrain.pose.heading.plus(Math.PI).inverse().times(
                        new Vector2d(-driveTrain.gamepadInput.x, driveTrain.gamepadInput.y));
            }
        }

        driveTrain.driveNormal();
        safeWaitMilliSeconds(10);

    }

    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!teleOpMode.isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Method to convert linear map from gamepad1 and gamepad2 stick input to a cubic map
     *
     * @param stickInput input value of button stick vector
     * @return Cube of the stick input reduced to 25% speed
     */
    public double limitStick(double stickInput) {
        return (stickInput * stickInput * stickInput * 0.50); //0.25
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
        acceleration_factor = 1.0 + 1.0 * rightTriggerValue;
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

    /**
     * Methods to get the value of gamepad Left stick Y for Pan motion Y direction.
     * This is the method to apply any directional modifiers to match to the Y plane of robot.
     * For Hazmat Freight Frenzy Robot, Y direction needs to be inverted.
     *
     * @return gpGamepad1.left_stick_y
     */
    public double gp1GetLeftStickY() { return hzGamepad1.left_stick_y; }

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

    public double gp1GetRightStickY() {
        return hzGamepad1.right_stick_y;
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


}
