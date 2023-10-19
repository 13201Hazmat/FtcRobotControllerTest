/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "HazmatAutonomous Mode", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutonomousMode extends LinearOpMode {

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Vision vision;
    public Lights lights;

    //Static Class for knowing system state

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime startTimer = new ElapsedTime(MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", startPosition);

        // Initiate Camera on Init.
        vision.initTfod();

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        waitForStart();

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);

            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", vision.identifiedSpikeMarkLocation);
            telemetry.update();
        }

        //Stop Vision process
        /*if (visionPortal.getCameraState() != CAMERA_DEVICE_CLOSED) {
            //visionPortal.stopStreaming();
            visionPortal.close();
        }*/

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEFAULT);

            //Build parking trajectory based on last detected target by vision
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        Pose2d initPose = new Pose2d(0, 0, 0); // Starting Pose
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(12, 60, Math.toRadians(-90)); //Starting pose
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(vision.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(17, 35, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(48, 40, Math.toRadians(0));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(12, 32, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(48, 36,  Math.toRadians(0));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(7, 35, Math.toRadians(-135));
                        dropYellowPixelPose = new Pose2d(48, 32, Math.toRadians(0));
                        break;
                }
                midwayPose1 = new Pose2d(12, 50, Math.toRadians(-90));
                midwayPose2 = new Pose2d(36, 50, Math.toRadians(0));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(46, 60, Math.toRadians(0));
                break;

            case RED_RIGHT:
                initPose = new Pose2d(12, -60, Math.toRadians(90)); //Starting pose
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(vision.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(7, -35, Math.toRadians(135));
                        dropYellowPixelPose = new Pose2d(48, -32, Math.toRadians(0));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(12, -32, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(48, 36,  Math.toRadians(0));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(17, -35, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(48, -40, Math.toRadians(0));
                        break;
                }
                midwayPose1 = new Pose2d(12, -50, Math.toRadians(90));
                midwayPose2 = new Pose2d(36, -50, Math.toRadians(0));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(46, -60, Math.toRadians(0));
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-36, 60, Math.toRadians(-90)); //Starting pose
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(vision.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(-31, 15, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(48, 40, Math.toRadians(0));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-36, 8, Math.toRadians(90));
                        dropYellowPixelPose = new Pose2d(48, 36, Math.toRadians(0));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(-41, 15, Math.toRadians(135));
                        dropYellowPixelPose = new Pose2d(48, 32, Math.toRadians(0));
                        break;
                }
                intakeStack = new Pose2d(-54, 12,Math.toRadians(180));
                midwayPose1 = new Pose2d(-18, 12, Math.toRadians(0));
                midwayPose2 = new Pose2d(36, 12, Math.toRadians(0));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(46, 12, Math.toRadians(0));
                break;

            case RED_LEFT:
                initPose = new Pose2d(-36, -60, Math.toRadians(90)); //Starting pose
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(vision.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(-41, -15, Math.toRadians(135));
                        dropYellowPixelPose = new Pose2d(48, -32, Math.toRadians(0));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(-36, -8, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(48, -36, Math.toRadians(0));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(-31, -15, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(48, -40, Math.toRadians(0));
                        break;
                }
                intakeStack = new Pose2d(-54, -12,Math.toRadians(-180));
                midwayPose1 = new Pose2d(-18, -12, Math.toRadians(0));
                midwayPose2 = new Pose2d(36, -12, Math.toRadians(0));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(46, -12, Math.toRadians(0));
                break;
        }

        //drive.pose = initPose;

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //TODO : Code to drop Purple Pixel on Spike Mark
        safeWaitSeconds(1);

        //For Blue Right and Red Left, intake pixel from stack
        if (startPosition == START_POSITION.BLUE_RIGHT ||
            startPosition == START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());

            //TODO : Code to intake pixel from stack
            safeWaitSeconds(1);
        }

        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        //TODO: For Blue Right and Red Left, Add code to raise Stage Door to pass through
        safeWaitSeconds(1);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                        .waitSeconds(waitSecondsBeforeDrop)
                        .splineToLinearHeading(dropYellowPixelPose,Math.toRadians(90))
                        .build());


        //TODO : Code to drop Pixel on Backdrop
        safeWaitSeconds(1);

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());
    }


    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode");
            telemetry.addLine("---------------------------------------");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepad1.x){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepad1.y){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepad1.b){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepad1.a){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }
        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
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

        /* Create Vision */
        vision = new Vision(hardwareMap, telemetry);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, vision, telemetry);
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
            telemetry.addLine("Running Hazmat Autonomous Mode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            driveTrain.printDebugMessages();
            vision.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }


}   // end class
