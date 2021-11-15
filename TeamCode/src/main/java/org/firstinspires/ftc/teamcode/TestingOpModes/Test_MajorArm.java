package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test_MajorArm", group = "Test")
public class Test_MajorArm extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public MajorArm majorArm;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        //TODO: Declare subsystem to be tested
        majorArm = new MajorArm(hardwareMap);
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        majorArm.initMajorArm();

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

                //TODO: Add Test Code here
                if (gamepadTestController.getButtonXPress()){
                    majorArm.moveArmDownOne();
                }
                if (gamepadTestController.getButtonBPress()) {
                    majorArm.moveArmUpOne();
                }
                if(gamepadTestController.getButtonAPress()){
                    majorArm.moveArmPickupPosition();
                }
                if(gamepadTestController.getButtonYPress()){
                    majorArm.moveArmCapstonePosition();
                }
                if(gamepadTestController.getRightTriggerPress()){
                    majorArm.moveArmParkingPosition();
                }

                if (!gamepadTestController.getStartPersistent()) {
                    if (gamepadTestController.getLeftTriggerPress()) {
                        majorArm.moveMajorArmSlightlyDown();
                    }
                } else {
                    if (gamepadTestController.getLeftTriggerPress()) {
                        majorArm.moveMajorArmSlightlyUp();
                    }
                }

                if (majorArm.runArmToLevelState) {
                    majorArm.runArmToLevel(majorArm.ARM_MOTOR_POWER);
                }

                if(gamepadTestController.getRightBumperPress()){
                    majorArm.changeClawState();
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

        telemetry.addData("Major Arm Position : ",majorArm.getArmPosition());
        telemetry.addData("Major Claw State : ",majorArm.getMajorClawState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
