package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadDriveTrainController;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.VisionAprilTag;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "Hazmat TeleOp AprilTag", group = "00-Teleop")
public class TeleOpModeVisionAprilTag extends LinearOpMode {

    public GamepadController gamepadController;
    public GamepadDriveTrainController gamepadDriveTrainController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public VisionAprilTag visionAprilTagBack;
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

        // Initiate Camera on Init.
        visionAprilTagBack.initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        telemetry.addLine("Start Pressed");
        telemetry.update();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        gamepadDriveTrainController.start();

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadController.runByGamepadControl();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
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
        driveTrain = new DriveTrain(hardwareMap, new Pose2d(0,0,0), telemetry);
        driveTrain.driveType = DriveTrain.DriveType.ROBOT_CENTRIC;
        telemetry.addData("DriveTrain Initialized with Pose:",driveTrain.toStringPose2d(driveTrain.pose));
        telemetry.update();

        intake = new Intake(hardwareMap, telemetry);
        telemetry.addLine("Intake Initialized");
        telemetry.update();

        magazine = new Magazine(hardwareMap, telemetry);
        telemetry.addLine("Magazine Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap, telemetry);
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap, telemetry);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
        telemetry.update();

        launcher = new Launcher(hardwareMap, telemetry);
        telemetry.addLine("Launcher Initialized");
        telemetry.update();

        /* Create VisionAprilTag */
        visionAprilTagBack = new VisionAprilTag(hardwareMap, telemetry, "Webcam 2");
        telemetry.addLine("Vision April Tag Front Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadDriveTrainController = new GamepadDriveTrainController(gamepad1, driveTrain, this);
        telemetry.addLine("Gamepad DriveTrain Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, intake, magazine,
                outtakeSlides, outtakeArm, climber, launcher, telemetry, this);
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
            telemetry.addLine("Running Hazmat TeleOpMode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            driveTrain.printDebugMessages();
            intake.printDebugMessages();
            outtakeSlides.printDebugMessages();
            outtakeArm.printDebugMessages();
            climber.printDebugMessages();
            launcher.printDebugMessages();
            visionAprilTagBack.printdebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }

}
