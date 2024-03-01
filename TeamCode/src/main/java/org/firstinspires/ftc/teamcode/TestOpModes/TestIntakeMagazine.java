package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.OldAuto.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */

@Disabled
@TeleOp(name = "Test IntakeMagazine", group = "02-Test OpModes")
public class TestIntakeMagazine extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public Lights lights;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    public ElapsedTime intakeReverseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public boolean intakeReverseStarted = false;

    public final static double intakeMotorPowerDashboard = 1.0;

    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


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
                gamepadController.runByGamepadControl();

                if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
                    printDebugMessages();
                    telemetry.update();
                }

                //Adjust Intake Power for testing
                if (gamepadController.gp1GetSquarePress()) {
                    if (intake.intakeMotorPower >0) {
                        intake.intakeMotorPower -=0.01;
                    }
                }

                if (gamepadController.gp1GetCirclePress()) {
                    if (intake.intakeMotorPower <=1.0) {
                        intake.intakeMotorPower +=0.01;
                    }
                }

                if (!gamepadController.gp1GetStart()) {
                    if (gamepadController.gp1GetLeftBumperPress()) {
                       // intake.toggleStackIntake();
                    }
                } else {
                    intake.moveRollerHeight(Intake.STACK_INTAKE_LIFT_STATE.LIFTED);
                }

                if(gamepadController.gp1GetCrossPress()) {
                    intake.moveIntakeLiftDown();
                }

                if (gamepadController.gp1GetDpad_downPress()){
                    if (intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.REVERSING) {
                        if (intake.intakeMotorState != Intake.INTAKE_MOTOR_STATE.RUNNING) {
                            intake.startIntakeInward();
                        } else {
                            intake.stopIntake();
                        }
                    } else {
                        if (intake.intakeMotorPrevState == Intake.INTAKE_MOTOR_STATE.RUNNING) {
                            intake.startIntakeInward();
                        } else {
                            intake.stopIntake();
                        }
                    }
                }

                //TODO:REVERESE ONLY WHEN MAGAZINE CHANGES STATE FROM 1 PIXEL TO 2 PIXEL
                magazine.senseMagazineState();
                if(magazine.magazineState == Magazine.MAGAZINE_STATE.LOADED_TWO_PIXEL) {
                    intakeReverseStarted = true;
                    intakeReverseTimer.reset();
                    intake.reverseIntake();
                }

                if (intakeReverseStarted && intakeReverseTimer.time() > 300) {
                    intake.stopIntake();
                    intakeReverseStarted = false;
                }

                if (gamepadController.gp1GetDpad_up()) {
                    intake.reverseIntake();
                } else {
                    if (intake.intakeMotorState == Intake.INTAKE_MOTOR_STATE.REVERSING) {
                        intake.stopIntake();
                    }
                }

                if (gamepadController.gp1GetLeftTriggerPersistent()) {
                    //magazine.openMagazineDoor();
                } else  {
                    //magazine.closeMagazineDoor();
                }

            }
        }
        GameField.poseSetInAutonomous = false;
    }

    public void initSubsystems(){

        //telemetry.setAutoClear(false);

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

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
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
        //telemetry.setAutoClear(true);
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
            magazine.printDebugMessages();
            //visionAprilTagFront.printdebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }

}
