package org.firstinspires.ftc.teamcode.TestingOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Test Intake", group = "Test")
public class TestIntake extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public Intake intake;

    //public Vuforia Vuforia1;
    public Pose2d startPose = GameField.ORIGINPOSE;

    @Override
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);   /*Subsystem to be tested*/
        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        intake.initIntake();

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


                if (gamepadTestController.getDpad_downPress()) {
                    if(intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.RUNNING) {
                        intake.startForwardIntakeMotor();
                    }
                    else if(intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.STOPPED) {
                        intake.stopIntakeMotor();
                    }
                }

                //Reverse Intake motors and run - in case of stuck state)
                if (gamepadTestController.getDpad_upPress()) {
                    if (intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.REVERSING) {
                        intake.startReverseIntakeMotor();
                    } else if (intake.getIntakeMotorState() != Intake.INTAKE_MOTOR_STATE.STOPPED) {
                        intake.stopIntakeMotor();
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

        telemetry.addData("Intake State : ", intake.getIntakeMotorState());

        //Add logic for debug print Logic

        telemetry.update();

    }
}
