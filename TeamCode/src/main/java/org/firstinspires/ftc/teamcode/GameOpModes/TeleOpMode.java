package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.SystemState;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "Hazmat TeleOp", group = "00-Teleop")
public class TeleOpMode extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public OuttakeArm outtakeArm;
    public OuttakeSlides outtakeSlides;
    public Lights lights;

    //Static Class for knowing system state
    public static SystemState SystemState;

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
        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

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
                gamepadController.runByGamepadControl();

                if (gameTimer.time() > 80000 && gameTimer.time() < 90000) {
                    lights.setPatternEndGame();
                }

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
        driveTrain = new DriveTrain(hardwareMap);
        telemetry.addLine("DriveTrain Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap);
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        lights = new Lights(hardwareMap);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intakeArm, intakeSlides, outtakeArm, outtakeSlides, lights);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.getLocalizer().setPoseEstimate(startPose);
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

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            telemetry.addLine("=============");

            telemetry.addData("Intake Slides State", intakeSlides.intakeSlidesState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Intake Slides Left Position", intakeSlides.intakeMotorLeft.getCurrentPosition());
                telemetry.addData("Intake Slides Right Position", intakeSlides.intakeMotorRight.getCurrentPosition());
                telemetry.addData("Intake Slides Left Power", intakeSlides.intakeMotorLeft.getPower());
                telemetry.addData("Intake Slides Right Power", intakeSlides.intakeMotorRight.getPower());
                telemetry.addData("Intake Slides Left is busy", intakeSlides.intakeMotorLeft.isBusy());
                telemetry.addData("Intake Slides Right is busy", intakeSlides.intakeMotorRight.isBusy());
                telemetry.addData("Intake Slides Touch Sensor State", intakeSlides.intakeTouch.getState());
            }
            telemetry.addLine("=============");

            telemetry.addData("Intake Arm State", intakeArm.armState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Intake Arm Left Position", "%.2f", intakeArm.intakeArmServoLeft.getPosition());
                telemetry.addData("Intake Arm Right Position", "%.2f", intakeArm.intakeArmServoRight.getPosition());
            }

            telemetry.addData("Intake Wrist State", intakeArm.wristState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Intake Wrist Left Position", "%.2f", intakeArm.intakeWristServoLeft.getPosition());
                telemetry.addData("Intake Wrist Right Position", "%.2f", intakeArm.intakeWristServoRight.getPosition());
            }
            telemetry.addData("Intake Grip State", intakeArm.gripState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Intake Grip Servo Position", "%.2f", intakeArm.intakeGripServo.getPosition());
            }
            telemetry.addData("Intake Grip Color Sensor", intakeArm.senseIntakeCone());
            if (GameField.debugLevel != GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Intake Grip Sensor Distance", "%.2f", ((DistanceSensor)intakeArm.intakeGripColor).getDistance(DistanceUnit.MM));
            }

            telemetry.addLine("=============");

            telemetry.addData("Outtake Slides State", outtakeSlides.outtakeSlidesState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Outtake Slides Position", outtakeSlides.outtakeMotor.getCurrentPosition());
                telemetry.addData("Outtake Slides Power", outtakeSlides.outtakeMotor.getPower());
                telemetry.addData("Outtake Slides is busy", outtakeSlides.outtakeMotor.isBusy());
                telemetry.addData("Outtake Slides Touch Sensor State", outtakeSlides.outtakeTouch.getState());
            }

            telemetry.addData("Outtake Turret State", outtakeSlides.turretState);
            if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Turret Position", outtakeSlides.outtakeTurretServo.getPosition());
            }

            telemetry.addLine("=============");

            telemetry.addData("Outtake Arm State", outtakeArm.wristState);
            if (GameField.debugLevel != GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Outtake Wrist Servo Position", "%.2f", outtakeArm.outtakeWristServo.getPosition());
            }

            telemetry.addData("Outtake Grip State", outtakeArm.gripState);
            if (GameField.debugLevel != GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Outtake Grip Servo Position", "%.2f", outtakeArm.outtakeGripServo.getPosition());
            }
            telemetry.addData("Outtake Grip Color Sensor", outtakeArm.senseOuttakeCone());
            if (GameField.debugLevel != GameField.DEBUG_LEVEL.MAXIMUM) {
                telemetry.addData("Outtake Grip Sensor Distance", "%.2f", ((DistanceSensor)outtakeArm.outtakeGripColor).getDistance(DistanceUnit.MM));
            }

            telemetry.addLine("=============");

        }
        telemetry.update();
    }

}
