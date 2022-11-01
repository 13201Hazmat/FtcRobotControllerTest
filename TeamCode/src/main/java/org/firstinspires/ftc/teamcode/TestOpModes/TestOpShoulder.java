package org.firstinspires.ftc.teamcode.TestOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.Camera;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;

/**
 * TestOp Arm <BR>
 *
 * This code defines the ArmTeleOp mode is done by Hazmat Robot for Powerplay<BR>
 *
 */
@TeleOp(name = "TestOpShoulder", group = "TestOp")
public class TestOpShoulder extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Arm arm;
    public Hand hand;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;
    public Camera camera;

    //public Vuforia Vuforia1;

    //public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;


    @Override
    /**
     * Constructor for passing all the subsystems in order to make the subsystem be able to use
     * and work/be active
     */
    public void runOpMode() throws InterruptedException {

        /* Create Subsystem Objects*/
        //driveTrain = new DriveTrain(hardwareMap);
        arm = new Arm(hardwareMap);
        hand = new Hand(hardwareMap);
        shoulder = new Shoulder(hardwareMap);
        turret = new Turret(hardwareMap);
        lights = new Lights(hardwareMap);
        camera = new Camera(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, arm, hand, shoulder, turret, lights);

        GameField.playingAlliance= GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
        /* Get last position after Autonomous mode ended from static class set in Autonomous */
        /*
        if ( GameField.poseSetInAutonomous == true) {
            driveTrain.getLocalizer().setPoseEstimate(GameField.currentPose);
        } else {
            driveTrain.getLocalizer().setPoseEstimate(startPose);
        }
        */


        GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        //GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;


        /* Set Initial State of any subsystem when TeleOp is to be started*/

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();
        gameTimer.reset();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                // gamepadController.runByGamepadControl();

                shoulder.turnShoulderBrakeModeOn();

                //Extend the shoulder based on the right joystick
                if (gamepadController.gp2GetRightTrigger() > 0.2) {
                    shoulder.raiseShoulder(gamepadController.gp2GetRightTrigger());
                    shoulder.runShoulderToLevel(gamepadController.gp2GetRightTrigger());
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(gamepadController.gp2GetRightTrigger());
                        shoulder.runShoulderToLevelState = false;
                    }
                }

                //retract the arm based on the right joystick
                else if(gamepadController.gp2GetLeftTrigger() > 0.2) {
                    shoulder.lowerShoulder(gamepadController.gp2GetLeftTrigger());
                    shoulder.runShoulderToLevel(gamepadController.gp2GetLeftTrigger());
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(gamepadController.gp2GetLeftTrigger());
                        shoulder.runShoulderToLevelState = false;
                    }
                } else {
                    shoulder.turnShoulderBrakeModeOn();
                }


                //Move arm to low junction if x is pressed
                if (gamepadController.gp2GetButtonXPress()){
                    shoulder.moveToShoulderLowJunction();
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(arm.MED_POWER);
                        shoulder.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to the high junction position if gamepad b is pressed
                if (gamepadController.gp2GetButtonBPress()){
                    shoulder.moveToShoulderHighJunction();
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(arm.MED_POWER);
                        shoulder.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to ground junction if gamepad a is pressed
                if (gamepadController.gp2GetButtonAPress()){
                    shoulder.moveToShoulderGroundJunction();
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(arm.MED_POWER);
                        shoulder.runShoulderToLevelState = false;
                    }
                }

                //Moves arm to middle junction if y is pressed
                if (gamepadController.gp2GetButtonYPress()){
                    shoulder.moveToShoulderMidJunction();
                    if (shoulder.runShoulderToLevelState){
                        shoulder.runShoulderToLevel(arm.MED_POWER);
                        shoulder.runShoulderToLevelState = false;
                    }
                }


                if (DEBUG_FLAG) {
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
        telemetry.addData("DEBUG_LEVEL is : ", DEBUG_FLAG);
        telemetry.addData("Robot ready to start","");
        telemetry.addData("Arm Motor Position: ", shoulder.getShoulderPositionCount());
        telemetry.addData("Arm Motor State: ", shoulder.currentShoulderPosition);
        telemetry.addData("Arm Delta Value: ", shoulder.shoulderDeltaCount);

            /*
            telemetry.addData("Drive Mode : ", driveTrain.driveMode);
            telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);





            telemetry.addData("Arm Motor Power:", arm.getArmMotorPower);
            telemetry.addData("Arm Motor Position: ", arm.getArmMotorPosition);

            telemetry.addData("Camera State : ", camera.getCameraState);

            telemetry.addData("Wrist Servo Power : ", hand.getWristServoPower);
            telemetry.addData("Grips Servo Power : ", hand.getGripServoPower);
            telemetry.addData("Left Intake Servo Power : ", hand.getIntklServoPower);
            telemetry.addData("Right Intake Servo Power : ", hand.getIntkrServoPower);

            telemetry.addData("Right Motor Shoulder Power : ", shoulder.getRshMotorPower);
            telemetry.addData("Left Motor Shoulder Power: ", shoulder.getLshMotorPower);
            telemetry.addData("Right Motor Shoulder Position : ", shoulder.getRshMotorPositon);
            telemetry.addData("Left Motor Shoulder Position : ", shoulder.getLshMotorPosition);

            telemetry.addData("Turret Motor Power : ", turret.getTurretMotorPower);
            telemetry.addData("Turret Motor Position : ", turret.getTurretMotorPosition);


            telemetry.addData("Game Timer : ", gameTimer.time());


             */


        telemetry.update();

    }
}
