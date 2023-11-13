package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.VisionAprilTag;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "Test Climber", group = "02-Test OpModes")
public class TestClimber extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public VisionAprilTag visionAprilTagFront;
    public Climber climber;
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

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                //gamepadController.runByGamepadControl();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }

                /*if (gamepadController.gp1GetB()) {
                    climber.moveClimberSlidesUp();
                }


                if (gamepadController.gp1GetX()) {
                    climber.moveClimberSlidesDown();
                }


                if (climber.climberServoRunning = true && (!gamepadController.gp1GetB() || !gamepadController.gp1GetX())){
                    climber.stopClimberSlides();
                }*/

                if (gamepadController.gp1GetButtonBPress()) {
                    switch (climber.climberButtonState) {
                        case SAFE:
                            climber.climberClickTimer.reset();
                            climber.climberButtonState = Climber.CLIMBER_BUTTON_STATE.ARMED;
                            break;
                        case ARMED:
                            if (climber.climberClickTimer.time() < climber.CLIMBER_BUTTON_ARMED_THRESHOLD) {
                                climber.releaseClimber();
                                climber.climberButtonState = Climber.CLIMBER_BUTTON_STATE.RELEASED;
                            } else {
                                climber.climberClickTimer.reset();
                                climber.climberButtonState = Climber.CLIMBER_BUTTON_STATE.SAFE;
                            }
                    }
                }

                if (climber.climberServoState == Climber.CLIMBER_SERVO_STATE.CLIMBER_RELEASED &&
                        gamepadController.gp1GetY()){
                    climber.moveClimberMotor(Climber.CLIMBER_MOTOR_STATE.CLIMBED_STATE);
                }

                if (gamepadController.gp1GetA()){
                    climber.moveClimberMotor(Climber.CLIMBER_MOTOR_STATE.INITIAL_STATE);
                }
            }
        }
        GameField.poseSetInAutonomous = false;
    }



    public void initSubsystems(){

        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        //driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);

       /* driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        */

        /* Create VisionAprilTag */
        visionAprilTagFront = new VisionAprilTag(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("Vision April Tag Front Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new TestGamepadController(gamepad1, gamepad2, driveTrain, telemetry);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            //driveTrain.pose = GameField.currentPose;
            //driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            //driveTrain.pose = startPose;
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
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

           // driveTrain.printDebugMessages();
            //visionAprilTagFront.printdebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }

}
