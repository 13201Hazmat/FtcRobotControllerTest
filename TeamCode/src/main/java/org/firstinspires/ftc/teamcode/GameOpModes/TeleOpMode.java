package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.SystemState;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "TeleOp", group = "00-Teleop")
public class TeleOpMode extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Arm arm;
    public Hand hand;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;

    //Static Class for knowing system state
    public static SystemState SystemState;

    //public Vuforia Vuforia1;

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);


    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {
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

        hand = new Hand(hardwareMap);
        telemetry.addLine("Hand Initialized");
        telemetry.update();

        arm = new Arm(hardwareMap);
        telemetry.addLine("Arm Initialized, Pulled in completely");
        telemetry.addData("  - Arm Touch Sensor State", arm.armTouchSensor.getState());
        telemetry.update();

        shoulder = new Shoulder(hardwareMap);
        telemetry.addLine("Shoulder Initialized, Pushed down completely");
        telemetry.addData("  - Should Touch Sensor State: ", shoulder.shoulderTouchSensor.getState());
        telemetry.update();

        turret = new Turret(hardwareMap);
        telemetry.addLine("Turret Initialized, Set to middle");
        telemetry.addData("  - Turret Left Mag Sensor State: ", turret.turretLeftMagneticSensor.getState());
        telemetry.addData("  - Turret Center Mag Sensor State: ", turret.turretCenterMagneticSensor.getState());
        telemetry.addData("  - Turret Right Mag Sensor State: ", turret.turretRightMagneticSensor.getState());
        telemetry.update();

        lights = new Lights(hardwareMap);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, arm, hand, shoulder, turret, lights);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

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

        if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {

            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            //****** Drive debug ******
            //telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            //telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
            telemetry.addLine("=============");

            telemetry.addData("Arm State: ", arm.armState);
            telemetry.addData("Arm Motor Position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Power:", arm.armMotor.getPower());
            telemetry.addData("Arm Touch Sensor State", arm.armTouchSensor.getState());
            telemetry.addLine("=============");

            telemetry.addData("Wrist State : ", hand.wristState);
            telemetry.addData("Wrist Servo Position : ", hand.wristServo.getPosition());
            telemetry.addData("Grip State : ", hand.gripState);
            telemetry.addData("Grips Servo Position : ", hand.gripServo.getPosition());
            telemetry.addLine("=============");

            telemetry.addData("Shoulder State: ", shoulder.shoulderState);
            telemetry.addData("Should Touch Sensor State: ", shoulder.shoulderTouchSensor.getState());
            telemetry.addData("Left Motor Shoulder Position : ", shoulder.leftShoulderMotor.getCurrentPosition());
            telemetry.addData("Left Motor Shoulder Power: ", shoulder.leftShoulderMotor.getPower());
            telemetry.addData("Right Motor Shoulder Position : ", shoulder.rightShoulderMotor.getCurrentPosition());
            telemetry.addData("Right Motor Shoulder Power : ", shoulder.rightShoulderMotor.getPower());
            telemetry.addData("Shoulder Movement Direction : ",shoulder.shoulderMovementDirection);
            telemetry.addData("Shoulder Current Position : ",shoulder.shoulderCurrentPosition);
            telemetry.addData("Shoulder New Position : ",shoulder.shoulderNewPosition);
            telemetry.addLine("=============");

            telemetry.addData("Turret State : ", turret.turretMotorState);
            telemetry.addData("Turret Left Mag Sensor State: ", turret.turretLeftMagneticSensor.getState());
            telemetry.addData("Turret Center Mag Sensor State: ", turret.turretCenterMagneticSensor.getState());
            telemetry.addData("Turret Right Mag Sensor State: ", turret.turretRightMagneticSensor.getState());
            telemetry.addData("Turret Motor Position : ", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Turret Motor Power : ", turret.turretMotor.getPower());
            telemetry.addData("Turret Delta Count : ", turret.turretDeltaCount);
            telemetry.addLine("=============");
        }
        telemetry.update();
    }
}
