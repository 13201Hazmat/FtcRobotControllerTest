package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
@TeleOp(name = "TestArm", group = "00-Teleop")
public class TestArm extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Arm arm;
    public Hand hand;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;

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

        GameField.playingAlliance= GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;

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

                // gamepadController.runByGamepadControl();

                arm.turnArmBrakeModeOn();

                //Extend the arm based on the right joystick
                if (gamepadController.gp2GetRightStickX() > 0.2) {
                    arm.extendArm(gamepadController.gp2GetRightStickX());
                    arm.runShoulderToLevel(gamepadController.gp2GetRightStickX());
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(gamepadController.gp2GetRightStickX());
                        arm.runShoulderToLevelState = false;
                    }
                }

                //retract the arm based on the right joystick
                else if(gamepadController.gp2GetRightStickX() < -0.2) {
                    arm.retractArm(gamepadController.gp2GetRightStickX());
                    arm.runShoulderToLevel(gamepadController.gp2GetRightStickX());
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(gamepadController.gp2GetRightStickX());
                        arm.runShoulderToLevelState = false;
                    }
                } else {
                    arm.turnArmBrakeModeOn();
                }


                //Move arm to low junction if x is pressed
                if (gamepadController.gp2GetButtonXPress()){
                    arm.moveToArmLowJunction();
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(arm.MED_POWER);
                        arm.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to the high junction position if gamepad b is pressed
                if (gamepadController.gp2GetButtonBPress()){
                    arm.moveToArmHighJunction();
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(arm.MED_POWER);
                        arm.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to ground junction if gamepad a is pressed
                if (gamepadController.gp2GetButtonAPress()){
                    arm.moveToArmGroundJunction();
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(arm.MED_POWER);
                        arm.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to middle junction if y is pressed
                if (gamepadController.gp2GetButtonYPress()){
                    arm.moveToArmMidJunction();
                    if (arm.runShoulderToLevelState){
                        arm.runShoulderToLevel(arm.MED_POWER);
                        arm.runShoulderToLevelState = false;
                    }
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

            telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
            telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            telemetry.addData("startPose : ", startPose);

            //****** Drive debug ******
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);

            telemetry.addData("Arm Motor Position: ", arm.getArmPositionCount());
            telemetry.addData("Arm Motor Power:", arm.armmotor.getPower());

            telemetry.addData("Wrist Servo Position : ", hand.wristServo.getPosition());
            telemetry.addData("Grips Servo Position : ", hand.gripServo.getPosition());
            telemetry.addData("Left Intake Servo Power : ", hand.intakeLeftServo.getPosition());
            telemetry.addData("Right Intake Servo Power : ", hand.intakeRightServo.getPosition());

            telemetry.addData("Left Motor Shoulder Position : ", shoulder.leftShoulderMotor.getCurrentPosition());
            telemetry.addData("Left Motor Shoulder Power: ", shoulder.leftShoulderMotor.getPower());
            telemetry.addData("Right Motor Shoulder Position : ", shoulder.rightShoulderMotor.getPower());
            telemetry.addData("Right Motor Shoulder Power : ", shoulder.rightShoulderMotor.getPower());

            telemetry.addData("Turret Motor Position : ", turret.turretMotor.getCurrentPosition());
            telemetry.addData("Turret Motor Power : ", turret.turretMotor.getPower());
            
            telemetry.addData("Game Timer : ", gameTimer.time());
        }

        telemetry.update();

    }
}
