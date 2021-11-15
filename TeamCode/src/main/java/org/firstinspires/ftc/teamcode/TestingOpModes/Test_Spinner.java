package org.firstinspires.ftc.teamcode.TestingOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Spinner", group = "Test")
public class Test_Spinner extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public Spinner spinner;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //TODO: Declare subsystem to be tested
        spinner= new Spinner(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        spinner.initSpinner();

        selectGamePlan();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadTestController.runByGamepadControl();

                if (!gamepadTestController.getStartPersistent()) { //Normal condition, start not pressed
                    if (gamepadTestController.getLeftBumperPress()) {
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
                    if (gamepadTestController.getLeftBumperPress()) {
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

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();

        //Add logic to select autonomous mode based on keypad entry
        while (!isStopRequested()) {

            if (gamepadTestController.getButtonBPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                GameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (gamepadTestController.getButtonXPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                GameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }
        telemetry.update();
        safeWait(200);

    }

    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            //Wait
        }
    }


    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("Spinner State : ", spinner.getSpinnerMotorState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
