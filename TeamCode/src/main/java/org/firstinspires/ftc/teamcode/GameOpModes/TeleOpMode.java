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

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap);
        shoulder = new Shoulder(hardwareMap);
        turret = new Turret(hardwareMap);
        lights = new Lights(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, arm, hand, shoulder, turret, lights);

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.getLocalizer().setPoseEstimate(startPose);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;


        /* Set Initial State of any subsystem when TeleOp is to be started*/

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

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
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel == GameField.DEBUG_LEVEL.MAXIMUM) {

            telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            telemetry.addData("startPose : ", startPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);

            telemetry.addData("Arm State: ", arm.armMotorState);
            telemetry.addData("Arm Motor Position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Power:", arm.armMotor.getPower());

            telemetry.addData("Wrist State : ", hand.wristState);
            telemetry.addData("Wrist Servo Position : ", hand.wristServo.getPosition());
            telemetry.addData("Grip State : ", hand.gripState);
            telemetry.addData("Grips Servo Position : ", hand.gripServo.getPosition());
            telemetry.addData("Left Intake Servo Power : ", hand.intakeLeftServo.getPosition());
            telemetry.addData("Right Intake Servo Power : ", hand.intakeRightServo.getPosition());

            telemetry.addData("Shoulder State: ", shoulder.shoulderState);
            telemetry.addData("Left Motor Shoulder Position : ", shoulder.leftShoulderMotor.getCurrentPosition());
            telemetry.addData("Left Motor Shoulder Power: ", shoulder.leftShoulderMotor.getPower());
            telemetry.addData("Right Motor Shoulder Position : ", shoulder.rightShoulderMotor.getCurrentPosition());
            telemetry.addData("Right Motor Shoulder Power : ", shoulder.rightShoulderMotor.getPower());

            telemetry.addData("Turret State : ", turret.turretMotorState);
            telemetry.addData("Turret Motor Position : ", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Turret Motor Power : ", turret.turretMotor.getPower());
            telemetry.addData("Turret Delta Count : ", turret.turretDeltaCount);
            
            telemetry.addData("Game Timer : ", gameTimer.time());
        }

        telemetry.update();

    }
}
