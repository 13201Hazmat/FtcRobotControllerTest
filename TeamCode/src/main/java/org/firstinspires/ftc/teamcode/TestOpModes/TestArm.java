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
@TeleOp(name = "TestArm", group = "Testing")
public class TestArm extends LinearOpMode {

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

                //Keep shoulder Brake on for Arm testing
                shoulder.turnShoulderBrakeModeOn();

                //Move arm based on Left Stick on Gamepad 2
                if ((gamepadController.gp2GetLeftStickY() >= 0.2) ||
                        (gamepadController.gp2GetLeftStickY() <= -0.2)) {
                    arm.modifyArmLength(Math.pow(-gamepadController.gp2GetLeftStickY() * 1.25 - 0.25, 3));
                }

                //Move arm to low junction if Gamepad 2 X is pressed
                if (gamepadController.gp2GetButtonXPress()){
                    arm.moveArmToLowJunction();
                }

                //Moves arm to the high junction if Gamepad 2 B is pressed
                if (gamepadController.gp2GetButtonBPress()){
                    arm.moveArmToHighJunction();
                }

                //Moves arm to pickup / ground junction if Gamepad 2 B is pressed
                if (gamepadController.gp2GetButtonAPress()){
                    arm.moveArmToPickUp();
                }

                //Moves arm to middle junction if Gamepad 2 Y is pressed
                if (gamepadController.gp2GetButtonYPress()){
                    arm.moveArmToMediumJunction();
                }

                // Run Arm motor if position is changed
                if (arm.runArmToLevelState) {
                    arm.runArmToLevel(arm.ARM_POWER);
                }

                //Keep shoulder Brake on for Arm testing
                shoulder.turnShoulderBrakeModeOn();

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
            telemetry.addData("Arm State: ", arm.armMotorState);
            telemetry.addData("Arm Delta Count : ", arm.armDeltaCount);
            telemetry.addData("Arm Motor Position: ", arm.armMotor.getCurrentPosition());
            telemetry.addData("Arm Motor Power:", arm.armMotor.getPower());
            telemetry.addData("Arm Brake Mode:", arm.armMotor.getZeroPowerBehavior());
            telemetry.addData("Left Shoulder Position:", shoulder.leftShoulderMotor.getCurrentPosition());
            telemetry.addData("Left Shoulder Brake Mode:", shoulder.leftShoulderMotor.getZeroPowerBehavior());
            telemetry.addData("Right Shoulder Position:", shoulder.rightShoulderMotor.getCurrentPosition());
            telemetry.addData("Right Shoulder Brake Mode:", shoulder.rightShoulderMotor.getZeroPowerBehavior());
        }

        telemetry.update();

    }
}
