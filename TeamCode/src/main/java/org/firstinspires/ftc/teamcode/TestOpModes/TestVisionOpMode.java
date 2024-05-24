package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@Disabled
@TeleOp(name = "Hazmat Test Vision", group = "02-Test OpModes")
public class TestVisionOpMode extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public VisionDual vision;
    public Lights lights;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_TELEOP;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {
            if (opModeInInit()) {
                telemetry.addData("DS preview on/off","3 dots, Camera Stream");
                telemetry.addLine();
                telemetry.addLine("----------------------------------------");
                telemetry.addLine(" Press A for WEBCAM1_APRILTAG");
                telemetry.addLine(" Press B for WEBCAM2_APRILTAG");
                telemetry.addLine(" Press X for WEBCAM1_TFOD");
                telemetry.addLine(" Press Y for WEBCAM2_TFOD");
            }

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            vision.activateDoubleVision();

            while (opModeIsActive()) {
                gamepadController.runByGamepadControl();

                vision.telemetryCurrentVisionPortal();

                if (gamepadController.gp1GetCrossPress()) {
                    vision.switchWebcamAction(VisionDual.VISION_TYPE.WEBCAM1_APRILTAG);
                }

                if (gamepadController.gp1GetCirclePress()) {
                    vision.switchWebcamAction(VisionDual.VISION_TYPE.WEBCAM2_APRILTAG);
                }


                if (gamepadController.gp1GetSquarePress()) {
                    vision.switchWebcamAction(VisionDual.VISION_TYPE.WEBCAM1_TFOD);
                }

                if (gamepadController.gp1GetTrianglePress()) {
                    vision.switchWebcamAction(VisionDual.VISION_TYPE.WEBCAM2_TFOD);
                }

                /*if (gamepadController.gp1GetLeftBumper()) {*/
                    vision.streamCurrentVisionPortal();
                /*} else {
                    vision.stopWebcamStreaming();
                }*/

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    //printDebugMessages();
                    telemetry.update();
                }
            }
        }
        vision.stopWebcamStreaming();
        vision.deactivateVision();
        GameField.poseSetInAutonomous = false;
    }

    public void initSubsystems(){

        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        /* Create Vision */
        vision = new VisionDual(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        //gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, vision, telemetry);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.pose = GameField.currentPose;
            //driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.pose = startPose;
            //driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {

            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            driveTrain.printDebugMessages();
            //vision.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }
}
