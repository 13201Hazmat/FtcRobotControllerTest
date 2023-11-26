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

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.VisionTfod;

/**
 * Hazmat Autonomous
 */
@Disabled
@Autonomous(name = "HazmatAutonomous Mode 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutonomousMode1 extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public VisionTfod visionTfodFront;
    public Lights lights;
    public OuttakeController outtakeController;
    public TelemetryPacket telemetryPacket = new TelemetryPacket();

    //Static Class for knowing system state

    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT

    }
    public static START_POSITION startPosition;

    public enum AUTO_OPTION{
        FULL_AUTONOMOUS,
        PRELOAD_AND_PARK
    }
    public static AUTO_OPTION autoOption;

    public enum PATHWAY_OPTION{
        RIGGING_SPIKEMARK,
        RIGGING_WALL,
        STAGEDOOR
    }
    public static PATHWAY_OPTION pathwayOption;

    public int pixelLoopCount = 2;

    public Pose2d startPose = GameField.ORIGINPOSE;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime startTimer = new ElapsedTime(MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();
        telemetry.addData("Selected Starting Position", GameField.startPosition);

        // Initiate Camera on Init.
        visionTfodFront.initTfod();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
        //waitForStart();

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("Selected Pathway", pathwayOption);
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            visionTfodFront.runTfodTensorFlow();
            telemetry.addData("Vision identified Parking Location", visionTfodFront.identifiedSpikeMarkLocation);
            telemetry.addData("Pixel Loops count", pixelLoopCount);
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
        Pose2d moveBeyondTrussPose = new Pose2d(0,0,0);
        Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
        Pose2d midwayPose1 = new Pose2d(0,0,0);
        Pose2d midwayPose1a = new Pose2d(0,0,0);
        Pose2d intakeStack = new Pose2d(0,0,0);
        Pose2d midwayPose2 = new Pose2d(0,0,0);
        Pose2d dropYellowPixelPose = new Pose2d(0, 0, 0);
        Pose2d parkPose = new Pose2d(0, 0, 0);
        double waitSecondsBeforeDrop = 0;
        MecanumDrive drive = new MecanumDrive(hardwareMap, initPose);

        initPose = new Pose2d(0, 0, Math.toRadians(0)); //Starting pose
        moveBeyondTrussPose = new Pose2d(15,0,0);

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(23, 36, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(33, 3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(30, 36,  Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(30, -9, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(37, 36, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(14, 13, Math.toRadians(-45));
                waitSecondsBeforeDrop = 2000; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, 30, Math.toRadians(-90));
                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(30, 9, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(21, -36, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(33, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -36,  Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(14, -13, Math.toRadians(45));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(8, -30, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(27, 9, Math.toRadians(45));
                        dropYellowPixelPose = new Pose2d(27, 86, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(33, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(34, 86, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(26, -8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(43, 86, Math.toRadians(-90));
                        break;
                }
                midwayPose1 = new Pose2d(8, -8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, -18, Math.toRadians(-90));
                intakeStack = new Pose2d(52, -19,Math.toRadians(-90));
                midwayPose2 = new Pose2d(52, 62, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2000; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(50, 84, Math.toRadians(-90));
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch(visionTfodFront.identifiedSpikeMarkLocation){
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(26, 8, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(37, -86, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(33, -3, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(29, -86, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27, -9, Math.toRadians(-45));
                        dropYellowPixelPose = new Pose2d(21, -86, Math.toRadians(90));
                        break;
                }
                midwayPose1 = new Pose2d(8, 8, Math.toRadians(0));
                midwayPose1a = new Pose2d(18, 18, Math.toRadians(90));
                intakeStack = new Pose2d(52, 19,Math.toRadians(90));
                midwayPose2 = new Pose2d(52, -62, Math.toRadians(90));
                waitSecondsBeforeDrop = 2000; //TODO: Adjust time to wait for alliance partner to move from board
                parkPose = new Pose2d(46, -84, Math.toRadians(90));
                break;
        }

        //drive.pose = initPose;

        //Move robot to dropPurplePixel based on identified Spike Mark Location
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                        .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                        .build());

        //Drop Purple Pixel on Spike Mark
        //Move robot to midwayPose1
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(midwayPose1.position, midwayPose1.heading)
                        .build());

        telemetry.addData("Expected midwayPose1", midwayPose1.log());
        telemetry.addData("    Actual", drive.pose);
        telemetry.update();
        safeWaitMilliSeconds(500);


        //For Blue Right and Red Left, intake pixel from stack
        if (GameField.startPosition == GameField.START_POSITION.BLUE_RIGHT ||
                GameField.startPosition == GameField.START_POSITION.RED_LEFT) {
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose1a.position, midwayPose1a.heading)
                            .strafeToLinearHeading(intakeStack.position, intakeStack.heading)
                            .build());
            safeWaitMilliSeconds(500);


            //Code to intake pixel from stack
            intakePixelLevel5FromStack();

            //Move robot to midwayPose2 and to dropYellowPixelPose
            Actions.runBlocking(
                    drive.actionBuilder(drive.pose)
                            .strafeToLinearHeading(midwayPose2.position, midwayPose2.heading)
                            .build());
            safeWaitMilliSeconds(1000);

        }

        safeWaitMilliSeconds(waitSecondsBeforeDrop);

        //Move robot to midwayPose2 and to dropYellowPixelPose
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .setReversed(true)
                        .splineToLinearHeading(dropYellowPixelPose,0)
                        //.strafeToLinearHeading(dropPurplePixelPose.position,dropPurplePixelPose.heading)
                        .build());
        safeWaitMilliSeconds(200);


        //Code to drop Pixel on Backdrop
        dropPixelInBackdrop();

        //Move robot to park in Backstage
        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .strafeToLinearHeading(parkPose.position, parkPose.heading)
                        //.splineToLinearHeading(parkPose,0)
                        .build());

        moveOuttakeToEndState();
        safeWaitMilliSeconds(2000);
    }

    public void intakePixelLevel5FromStack() {
        intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
        intake.startIntakeInward();
        safeWaitMilliSeconds(200);
        intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_5);
        safeWaitMilliSeconds(1000);
        intake.stopIntakeMotor();
        intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_INIT_AUTO);
    }

    public void dropPixelInBackdrop() {
        outtakeController.moveTravelToReadyForTransfer();
        while (!outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER) &&
                !outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.READY_FOR_TRANSFER)) {
            safeWaitMilliSeconds(50);
        }
        outtakeController.moveReadyForTransferToTransfer();
        safeWaitMilliSeconds(500);
        outtakeController.moveTransferToPickup();
        safeWaitMilliSeconds(200);
        outtakeController.movePickupToTransfer();
        safeWaitMilliSeconds(100);
        outtakeController.moveTransferToReadyForTransfer();
        safeWaitMilliSeconds(200);
        outtakeController.moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LEVEL_MID);
        safeWaitMilliSeconds(500);
        outtakeArm.dropOnePixel();
        safeWaitMilliSeconds(200);
        if (GameField.startPosition == GameField.START_POSITION.RED_LEFT ||
                GameField.startPosition == GameField.START_POSITION.BLUE_RIGHT) {
            outtakeArm.dropOnePixel();
            safeWaitMilliSeconds(200);
        }
        outtakeController.moveDropToTravel();
        intake.moveRollerHeight(Intake.INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED);
        safeWaitMilliSeconds(2000);
    }

    public void moveOuttakeToEndState(){
        //End state of Outtake has to be Travel
        switch (outtakeSlides.outtakeSlidesState) {
            case MIN_RETRACTED:
            case TRANSFER:
            case PICKUP:
                outtakeController.movePickupToTransfer();
                outtakeController.moveTransferToReadyForTransfer();
                while(outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER)) {
                    safeWaitMilliSeconds(50);
                }
                outtakeController.moveReadyForTransferToTravel();
                break;
            case READY_FOR_TRANSFER:
            case TRAVEL:
            case DROP_LOWEST:
            case DROP_LOW_LINE:
            case DROP_BELOW_MID:
            case DROP_LEVEL_MID:
            case DROP_BELOW_HIGH:
            case DROP_LEVEL_HIGH:
            case DROP_HIGHEST:
            case MAX_EXTENDED:
            case RANDOM:
                outtakeController.moveDropToTravel();
                break;
        }
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Red Right  ", "(A / X)");
            if(gamepadController.gp1GetButtonXPress()){
                startPosition = START_POSITION.BLUE_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                startPosition = START_POSITION.BLUE_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                startPosition = START_POSITION.RED_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonAPress()){
                startPosition = START_POSITION.RED_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            telemetry.update();
        }

        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addLine("Select Pathway option");
            telemetry.addData("  Through the spike mark Rigging  ", "X / ▢");
            telemetry.addData("Through the wall Rigging ", "Y / Δ");
            telemetry.addData("Through the Stage Door", "B / O ");
            if(gamepadController.gp1GetButtonXPress()){
                pathwayOption = PATHWAY_OPTION.RIGGING_SPIKEMARK;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                break;
            }
            telemetry.update();
        }
        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addLine("Select Auto Options");
            telemetry.addData("    Full autonomous                ","X / ▢");
            telemetry.addData("    Drop Preload and Park   ","Y / Δ");
            if(gamepadController.gp1GetButtonXPress()){
                autoOption = AUTO_OPTION.FULL_AUTONOMOUS;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                autoOption = AUTO_OPTION.PRELOAD_AND_PARK;
                break;
            }
            telemetry.update();
        }
        if(autoOption == AUTO_OPTION.PRELOAD_AND_PARK){
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            pixelLoopCount = 0;
        }
        if(autoOption == AUTO_OPTION.FULL_AUTONOMOUS){
            pixelLoopCount = 2;
            while(!isStopRequested()){
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addLine("Select number of cones to pick and drop");
                telemetry.addLine("(Use dpad up to increase and dpad down to reduce)");
                telemetry.addLine("(Once final, press X / Square to finalize)");

                telemetry.addData("    dropConeCount (including preloaded, Max 2)", pixelLoopCount);

                if (gamepadController.gp1GetDpad_upPress()) {
                    pixelLoopCount += 1;
                    if (pixelLoopCount > 2) {
                        pixelLoopCount = 2;
                    }
                }
                if (gamepadController.gp1GetDpad_downPress()) {
                    pixelLoopCount -= 1;
                    if (pixelLoopCount < 1) {
                        pixelLoopCount = 1;
                    }
                }

                if(gamepadController.gp1GetButtonXPress()){
                    telemetry.addData("Selected number of loop Pixels excluding preloaded", pixelLoopCount);
                    break;
                }
                telemetry.update();
            }
        }


        telemetry.clearAll();
    }

    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitMilliSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
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

        intake = new Intake(hardwareMap, telemetry);
        telemetry.addLine("Intake Initialized");
        telemetry.update();

        magazine = new Magazine(hardwareMap, telemetry);
        telemetry.addLine("Magazine Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap, telemetry);
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap, telemetry);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        climber = new Climber(hardwareMap, telemetry);
        telemetry.addLine("Climber Initialized");
        telemetry.update();

        launcher = new Launcher(hardwareMap, telemetry);
        telemetry.addLine("Launcher Initialized");
        telemetry.update();

        /* Create Vision */
        visionTfodFront = new VisionTfod(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("VisionTfod Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        outtakeController = new OuttakeController(this.outtakeSlides, this.outtakeArm, this);
        telemetry.addLine("Outtake Controller Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, intake, magazine,
                outtakeSlides, outtakeArm, climber, launcher, lights, telemetry, this);
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
            lights.printDebugMessages();
        }
        telemetry.update();
    }


}   // end class
