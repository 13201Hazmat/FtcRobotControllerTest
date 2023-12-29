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
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.VisionAprilTag;


/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@Disabled
@TeleOp(name = "Test Outtake", group = "02-Test OpModes")
public class TestOuttake extends LinearOpMode {

    public TestGamepadController gamepadController;
    public DriveTrain driveTrain;
    public VisionAprilTag visionAprilTagFront;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Lights lights;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime comboTransferTimer = new ElapsedTime(MILLISECONDS);
    public boolean comboTransferActivated = false;
    public enum ComboPressedState {
        COMBO_INACTIVE,
        MOVE_TO_READY_TO_TRANSFER,
        WAIT_TILL_READY_TO_TRANSFER,
        MOVE_TO_TRANSFER,
        WAIT_TILL_TRANFERRED,
        MOVE_BACK_TO_READY_TO_TRANSFER,
        WAIT_TILL_BACK_TO_READY_TO_TRANSFER,
        MOVE_TO_DROP_BELOW_LOW
    }
    public ComboPressedState comboPressedState = ComboPressedState.COMBO_INACTIVE;

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

                //TODO : Move to transfer before rotating arm if starting in DROP_BELOW_LOW
                if (gamepadController.gp2GetSquarePress()){
                    outtakeArm.closeGrip();
                    if (outtakeSlides.outtakeSlidesState != OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST) {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOWEST);
                    } else {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOW_LINE);
                    }

                    outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
                }

                if (gamepadController.gp2GetTrianglePress()){
                    outtakeArm.closeGrip();
                    if (outtakeSlides.outtakeSlidesState != OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID) {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_BELOW_MID);
                    } else {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_MID);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LEVEL_MID);
                    }

                    outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
                }

                if (gamepadController.gp2GetCrossPress()){
                    outtakeArm.closeGrip();
                    if (outtakeSlides.outtakeSlidesState != OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_HIGH) {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_HIGH);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_BELOW_HIGH);
                    } else {
                        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_HIGH);
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LEVEL_HIGH);
                    }

                    outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
                }

                //if (!gamepadController.gp2GetStart()) {
                    if (gamepadController.gp2GetCirclePress()) {
                        if (outtakeSlides.outtakeSlidesState == OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER) {
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
                            while (outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER) &&
                                    outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)) {}
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.PICKUP);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.PICKUP);
                        } else {
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
                        }
                    }
                /*} else {
                    if (gamepadController.gp2GetButtonAPress()) {
                        outtakeArm.openGrip();
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                        outtakeSlides.manualResetOuttakeMotor();
                    }
                }*/

                //Combo Transfer
                if (gamepadController.gp2GetLeftBumperPress()) {
                    comboTransferActivated = true;
                }

                if (comboTransferActivated) {
                    switch (comboPressedState) {
                        case COMBO_INACTIVE:
                            comboPressedState = ComboPressedState.MOVE_TO_READY_TO_TRANSFER;
                            break;
                        case MOVE_TO_READY_TO_TRANSFER:
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
                            outtakeArm.openGrip();
                            comboPressedState = ComboPressedState.WAIT_TILL_READY_TO_TRANSFER;
                            break;
                        case WAIT_TILL_READY_TO_TRANSFER:
                            if (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER) &&
                                    outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER)) {
                                comboPressedState = ComboPressedState.MOVE_TO_TRANSFER;
                            }
                            break;
                        case MOVE_TO_TRANSFER:
                            //intake.stopIntakeMotor;
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.TRANSFER);
                            comboPressedState = ComboPressedState.WAIT_TILL_TRANFERRED;
                            break;
                        case WAIT_TILL_TRANFERRED:
                            if (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER) &&
                                    outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)) {
                                outtakeArm.closeGrip();
                                comboPressedState = ComboPressedState.MOVE_BACK_TO_READY_TO_TRANSFER;
                            }
                            break;
                        case MOVE_BACK_TO_READY_TO_TRANSFER:
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER);
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
                            comboPressedState = ComboPressedState.WAIT_TILL_BACK_TO_READY_TO_TRANSFER;
                            break;
                        case WAIT_TILL_BACK_TO_READY_TO_TRANSFER:
                            if (outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER) &&
                                    outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER)) {
                                comboPressedState = ComboPressedState.MOVE_TO_DROP_BELOW_LOW;
                            }
                            break;
                        case MOVE_TO_DROP_BELOW_LOW:
                            outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP_LOWEST);
                            outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.DROP);
                            outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST);
                            comboTransferActivated = false;
                            break;
                    }
                }

                //Open Outtake Grip
                if ((outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_HIGH) ||
                        outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_HIGH) ||
                        outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_MID) ||
                        outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID) ||
                        outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE) ||
                        outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST)) &&
                        outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP_HIGHEST)) {
                    if (gamepadController.gp2GetRightBumper()) {
                        outtakeArm.dropOnePixel();
                    }
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

        /* Create VisionAprilTag */
        visionAprilTagFront = new VisionAprilTag(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("Vision April Tag Front Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap, telemetry);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        outtakeArm= new OuttakeArm(hardwareMap, telemetry);
        telemetry.addLine("OuttakeArm Initialized");
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
            //visionAprilTagFront.printdebugMessages();
            outtakeSlides.printDebugMessages();
            outtakeArm.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }

}
