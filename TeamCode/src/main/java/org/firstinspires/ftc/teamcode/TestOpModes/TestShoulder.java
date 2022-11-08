package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 * This code defines the TeleOp mode is done by Hazmat Robot for Freight Frenzy<BR>
 *
 */
@TeleOp(name = "TestShoulder", group = "00-Teleop")
public class TestShoulder extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Arm arm;
    public Hand hand;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;

    //public Vuforia Vuforia1;

    //public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);


    @Override
    /*
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        //driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        //hand = new Hand(hardwareMap);
        shoulder = new Shoulder(hardwareMap);
        turret = new Turret(hardwareMap);
        lights = new Lights(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, arm, hand, shoulder, turret, lights);

        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        if ( GameField.poseSetInAutonomous) {
            //driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            //driveTrain.getLocalizer().setPoseEstimate(startPose);
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
                //gamepadController.runByGamepadControl();

                //Keep Arm fixed when testing shoulder
                arm.turnArmBrakeModeOn();

                //Extend the shoulder based on the Gamepad 2 left and right trigger
                if (gamepadController.gp2GetRightTrigger() > 0.2) {
                    shoulder.raiseShoulder(Math.pow(gamepadController.gp2GetRightTrigger() * 1.25 - 0.25, 3));
                } else if(gamepadController.gp2GetLeftTrigger() > 0.2) { //retract the arm based on the right joystick
                    shoulder.lowerShoulder(Math.pow(gamepadController.gp2GetLeftTrigger() * 1.25 - 0.25, 3));
                }

                //Move shoulder to low junction if Gamepad 2 X is pressed
                if (gamepadController.gp2GetButtonXPress()){
                    shoulder.moveToShoulderLowJunction();
                }

                //Moves shoulder to the high junction if Gamepad 2 B is pressed
                if (gamepadController.gp2GetButtonBPress()){
                    shoulder.moveToShoulderHighJunction();
                }

                //Moves shoulder to ground junction if Gamepad 2 A is pressed
                if (gamepadController.gp2GetButtonAPress()){
                    shoulder.moveToShoulderPickupWhileFacingFoward();
                }

                //Moves shoulder to middle junction if Gamepad 2 Y is pressed
                if (gamepadController.gp2GetButtonYPress()){
                    shoulder.moveToShoulderMidJunction();
                }

                //Run Shoulder motors if position is changed
                if (shoulder.runShoulderToLevelState){
                    shoulder.runShoulderToLevel(shoulder.SHOULDER_POWER);
                }

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
            telemetry.addData("Shoulder State: ", shoulder.shoulderState);
            telemetry.addData("Shoulder Delta Count : ", shoulder.shoulderDeltaCount);
            telemetry.addData("Left Motor Shoulder Position : ", shoulder.leftShoulderMotor.getCurrentPosition());
            telemetry.addData("Left Motor Shoulder Power: ", shoulder.leftShoulderMotor.getPower());
            telemetry.addData("Left Motor Shoulder Brake Mode: ", shoulder.leftShoulderMotor.getZeroPowerBehavior());
            telemetry.addData("Right Motor Shoulder Position : ", shoulder.rightShoulderMotor.getCurrentPosition());
            telemetry.addData("Right Motor Shoulder Power : ", shoulder.rightShoulderMotor.getPower());
            telemetry.addData("Right Motor Shoulder Brake Mode: ", shoulder.rightShoulderMotor.getZeroPowerBehavior());

        }

        telemetry.update();

    }
}
