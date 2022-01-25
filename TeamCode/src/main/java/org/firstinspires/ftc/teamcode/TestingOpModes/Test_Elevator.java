package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Elevator", group = "Test")
@Disabled
public class Test_Elevator extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public Elevator elevator;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //Declare subsystem to be tested
        elevator = new Elevator(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        elevator.initElevator();

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

                //Test Code
                if (gamepadTestController.getButtonAPress()){
                    if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_0) {
                        elevator.moveElevatorLevel0Position();
                    }
                }

                if (gamepadTestController.getButtonXPress()){
                    if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_1) {
                        elevator.moveElevatorLevel1Position();
                    }
                }

                if (gamepadTestController.getButtonYPress()){
                    if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_2) {
                        elevator.moveElevatorLevel2Position();
                    }
                }

                if (gamepadTestController.getButtonBPress()){
                    if (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_3) {
                        elevator.moveElevatorLevel3Position();

                    }
                }

                if (!gamepadTestController.getStartPersistent()) {
                    if (gamepadTestController.getLeftTriggerPress()) {
                        if ((elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_0) &&
                                (elevator.elevatorState != Elevator.ELEVATOR_STATE.LEVEL_1)) {
                            elevator.moveElevatorSlightlyDown();
                        }
                    }
                } else {
                    if (gamepadTestController.getLeftTriggerPress()) {
                        elevator.moveElevatorSlightlyUp();
                    }
                }

                if (elevator.runElevatorToLevelState){
                    elevator.runElevatorToLevel(elevator.motorPowerToRun);
                }

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
        GameField.poseSetInAutonomous = false;
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

        telemetry.addData("Elevator State : ", elevator.getElevatorState());
        telemetry.addData("Elevator Position Count : ", elevator.getElevatorPositionCount());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
