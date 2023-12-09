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
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
import org.firstinspires.ftc.teamcode.SubSystems.VisionSensor;
import org.firstinspires.ftc.teamcode.SubSystems.VisionTfod;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "HazmatAutonomous Mode 4", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutonomousMode5 extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public VisionSensor visionSensor;
    public VisionTfod visionTfodFront;
    public Lights lights;
    public OuttakeController outtakeController;
    public TelemetryPacket telemetryPacket = new TelemetryPacket();

    public MecanumDrive drive;

    //Static Class for knowing system state

    public enum AUTO_OPTION{
        FULL_AUTONOMOUS,
        PRELOAD_AND_PARK
    }
    public static AUTO_OPTION autoOption;

    public enum PATHWAY_OPTION{
        RIGGING_WALL,
        STAGEDOOR
    }
    public static PATHWAY_OPTION pathwayOption;

    public enum PARKING_OPTION{
        RIGGING_WALL,
        STAGEDOOR
    }

    public static PARKING_OPTION parkingOption;

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
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addData("Selected Pathway", pathwayOption);
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            visionTfodFront.runTfodTensorFlow();
            telemetry.addData("Vision identified SpikeMark Location", visionTfodFront.identifiedSpikeMarkLocation);
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

    Pose2d initPose= new Pose2d(0, 0, 0);
    Pose2d moveBeyondTrussPose = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPose = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPose = new Pose2d(0, 0, 0);
    Pose2d stageDoorStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayBackDropPose = new Pose2d(0, 0, 0);
    Pose2d wallStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayBackDropPose = new Pose2d(0, 0, 0);
    Pose2d stackPose = new Pose2d(0, 0, 0);
    Pose2d midwayStackPose = new Pose2d(0, 0, 0);
    Pose2d midwayBackDropPose = new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPose= new Pose2d(0, 0, 0);
    Pose2d beforeParkAfterDropYellowPixelPose = new Pose2d(0,0,0);
    Pose2d dropStackPixelPose = new Pose2d(0, 0, 0);
    Pose2d beforeParkAfterDropStackPixelPose = new Pose2d(0,0,0);
    Pose2d parkPoseStageDoor = new Pose2d(0, 0, 0);
    Pose2d parkPoseWall = new Pose2d(0, 0, 0);
    Pose2d parkPose = new Pose2d(0, 0, 0);


    Action trajInitToDropPurplePixel, trajDropPurplePixelToDropYellowPixel, trajDropYellowPixelToPark;
    Action trajDropYellowPixelPoseToMidwayBackDropPose, trajMidwayBackDropPoseToMidwayStackPose, trajAfterPurplePixelPoseToStackPose;
    Action trajStackPoseToMidwayStackPose, trajMidwayStackPoseToMidwayBackDropPose,
            trajMidwayBackDropPoseToDropStackPixelPose, trajDropStackPixelToPark;
    Action trajDropPurplePixelToAfterPurplePixelPose, trajAfterPurplePixelToMidwayStackPose;
    Action trajMidwayBackDropPoseToDropYellowPixekPose, trajDropYellowPixelPoseToDropStackPixelPose;


    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        moveBeyondTrussPose = new Pose2d(9, 0, 0); //x15

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionTfodFront.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(16.5, 9, Math.toRadians(32)); //22, 10.5, 32
                        dropYellowPixelPose = new Pose2d(18.7, 34.5, Math.toRadians(-90));//x20, y35.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(20,30, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(21, 36, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(21,32, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(31, 1, Math.toRadians(-30));
                        dropYellowPixelPose = new Pose2d(25, 34.5,  Math.toRadians(-90));//x24.5, y35.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(25,30, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(21, 36, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(21,32, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(28, -4, Math.toRadians(-30));
                        dropYellowPixelPose = new Pose2d(31.5, 34.5, Math.toRadians(-90)); //y=33.5, x35.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(33.5,30, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(37, 36, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(37,32, Math.toRadians(-90));
                        break;
                }
                afterPurplePixelPose = new Pose2d(8,-14,Math.toRadians(-30));
                wallStackPose = new Pose2d(13, -73, Math.toRadians(-90));
                wallMidwayStackPose = new Pose2d(2, -52, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(2, 18, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(49, -73, Math.toRadians(-90));
                stageMidwayStackPose = new Pose2d(49, -56, Math.toRadians(-90));
                stageMidwayBackDropPose = new Pose2d(49, 18, Math.toRadians(-90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPoseWall = new Pose2d(2.5, 27, Math.toRadians(-90)); //x2, x26
                parkPoseStageDoor = new Pose2d(48, 32, Math.toRadians(-90)); //x50, y30

                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionTfodFront.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(28, 4, Math.toRadians(30));
                        dropYellowPixelPose = new Pose2d(33, -35.5, Math.toRadians(90)); //x=33,y=-34
                        beforeParkAfterDropYellowPixelPose = new Pose2d(33,-30, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(37,-32,Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(31, -1, Math.toRadians(30));
                        dropYellowPixelPose = new Pose2d(26, -35.5,  Math.toRadians(90));//y-34, x26
                        beforeParkAfterDropYellowPixelPose = new Pose2d(26,-30, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(21, -36, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(21,-32,Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(24, -13, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(16.5, -35.5, Math.toRadians(90));//x18, y-34.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(16.5,-30, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(21, -36, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(21,-32,Math.toRadians(90));
                        break;
                }
                afterPurplePixelPose = new Pose2d(8,14,Math.toRadians(30));
                wallStackPose = new Pose2d(13, 73, Math.toRadians(90));
                wallMidwayStackPose = new Pose2d(2, 52, Math.toRadians(90));
                wallMidwayBackDropPose = new Pose2d(2, -18, Math.toRadians(90));
                stageDoorStackPose = new Pose2d(49, 73, Math.toRadians(90));
                stageMidwayStackPose = new Pose2d(49, 56, Math.toRadians(90));
                stageMidwayBackDropPose = new Pose2d(49, -18, Math.toRadians(90));
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                parkPoseWall = new Pose2d(2, -28, Math.toRadians(90));
                parkPoseStageDoor = new Pose2d(49, -28, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionTfodFront.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(28, 4, Math.toRadians(35));
                        dropYellowPixelPose = new Pose2d(22.7, 87, Math.toRadians(-90));//x24, y86
                        beforeParkAfterDropYellowPixelPose = new Pose2d(24,82, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(36, 89, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(36,85, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(19.5,-5, Math.toRadians(-90)); //x4, y1
                        stageMidwayStackPose = new Pose2d(49.7, -7, Math.toRadians(-90)); //x43, y8
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(26.4, 4.2, Math.toRadians(2)); //x31, y1, 30
                        dropYellowPixelPose = new Pose2d(24, 87,  Math.toRadians(-90)); //x26, y88.6
                        beforeParkAfterDropYellowPixelPose = new Pose2d(31.5,82, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(16, 89, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(16,85, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(18,-4,Math.toRadians(-30));
                        stageMidwayStackPose = new Pose2d(51.8, -15, Math.toRadians(-90)); //x49, y9, -90
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(25.5, -2.5, Math.toRadians(-52.5 )); //x15.8, y-6.5, -35
                        dropYellowPixelPose = new Pose2d(30, 87, Math.toRadians(-90)); //x31, 87
                        beforeParkAfterDropYellowPixelPose = new Pose2d(39,82, Math.toRadians(-90));
                        dropStackPixelPose = new Pose2d(36, 89, Math.toRadians(-90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(36,85, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(15,5,Math.toRadians(-90)); //x18, y-4, -30degrees
                        stageMidwayStackPose = new Pose2d(53, 7, Math.toRadians(-90)); //x44, y5, 0
                        break;
                }
                wallStackPose = new Pose2d(27, -18, Math.toRadians(-90)); //x26,-18
                wallMidwayStackPose = new Pose2d(2, 5, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(2, 73, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(51, -17.5, Math.toRadians(-90));//x51, y16
                stageMidwayBackDropPose = new Pose2d(51, 71, Math.toRadians(-90)); //x49, y73
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPoseWall = new Pose2d(2, 80.8, Math.toRadians(-90));//x3, y80.8
                parkPoseStageDoor = new Pose2d(46, 83.5, Math.toRadians(-90)); //x54.5, y77
                break;


            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionTfodFront.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(24, 8, Math.toRadians(37)); //x23, y5, 43
                        dropYellowPixelPose = new Pose2d(28, -91, Math.toRadians(90));//x30, y-90.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(34,-85, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(16, -89, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(36,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(4,1, Math.toRadians(90));
                        stageMidwayStackPose = new Pose2d(43, 8, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(27, 0, Math.toRadians(-5)); //x31, y-1. -30
                        dropYellowPixelPose = new Pose2d(22, -91,  Math.toRadians(90)); //x20,y=-90
                        beforeParkAfterDropYellowPixelPose = new Pose2d(27,-85, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(16, -89, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(36,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(18,4,Math.toRadians(-30));
                        stageMidwayStackPose = new Pose2d(49, 9, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(29, -4, Math.toRadians(-23)); //x30, y-2, -30
                        dropYellowPixelPose = new Pose2d(13, -91, Math.toRadians(90));//x14.5 y=-89
                        beforeParkAfterDropYellowPixelPose = new Pose2d(17,-85, Math.toRadians(90));
                        dropStackPixelPose = new Pose2d(36, -89, Math.toRadians(90));
                        beforeParkAfterDropStackPixelPose = new Pose2d(36,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(18,4,Math.toRadians(-30));
                        stageMidwayStackPose = new Pose2d(49, 9, Math.toRadians(90));
                        break;
                }
                wallStackPose = new Pose2d(22, 17, Math.toRadians(90)); //x23, x18, 90
                wallMidwayStackPose = new Pose2d(2, -5, Math.toRadians(90));
                wallMidwayBackDropPose = new Pose2d(2, -73, Math.toRadians(90));
                stageDoorStackPose = new Pose2d(48.5, 18, Math.toRadians(90));//x48.5, y18.7
                stageMidwayBackDropPose = new Pose2d(48.5, -73, Math.toRadians(90));//x49
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board
                parkPoseWall = new Pose2d(-1, -81, Math.toRadians(90)); //x12.2, y-85
                parkPoseStageDoor = new Pose2d(46, -79, Math.toRadians(90));//x58, y-78
                break;
        }

        if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                stackPose = wallStackPose;
                midwayStackPose = wallMidwayStackPose;
                midwayBackDropPose = wallMidwayBackDropPose;
        } else {
                stackPose = stageDoorStackPose;
                midwayStackPose = stageMidwayStackPose;
                midwayBackDropPose = stageMidwayBackDropPose;
        }


        if (parkingOption == PARKING_OPTION.RIGGING_WALL) {
            parkPose = parkPoseWall;
        } else {
            parkPose = parkPoseStageDoor;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToDropPurplePixel = drive.actionBuilder(drive.pose)
                //.strafeToLinearHeading(moveBeyondTrussPose.position, moveBeyondTrussPose.heading)
                //.strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                .splineToLinearHeading(dropPurplePixelPose,0)
                .build();

        //For RED_RIGHT & BLUE_LEFT
        trajDropPurplePixelToDropYellowPixel = drive.actionBuilder(dropPurplePixelPose)
                .setReversed(true)
                .splineToLinearHeading(dropYellowPixelPose, 0)
                .build();

        trajDropYellowPixelToPark = drive.actionBuilder(dropYellowPixelPose)
                .strafeToLinearHeading(beforeParkAfterDropYellowPixelPose.position, beforeParkAfterDropYellowPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();

        trajDropYellowPixelPoseToMidwayBackDropPose = drive.actionBuilder(dropYellowPixelPose)
                .strafeToLinearHeading(midwayBackDropPose.position, midwayBackDropPose.heading)
                .build();

        trajMidwayBackDropPoseToMidwayStackPose = drive.actionBuilder(midwayBackDropPose)
                .strafeTo(midwayStackPose.position)
                .build();

        trajAfterPurplePixelPoseToStackPose = drive.actionBuilder(afterPurplePixelPose)
                .strafeTo(stackPose.position)
                .build();

        trajStackPoseToMidwayStackPose = drive.actionBuilder(stackPose)
                .strafeTo(midwayStackPose.position)
                .build();

        trajMidwayStackPoseToMidwayBackDropPose = drive.actionBuilder(midwayStackPose)
                .strafeTo(midwayBackDropPose.position)
                .build();

        trajMidwayBackDropPoseToDropStackPixelPose = drive.actionBuilder(midwayBackDropPose)
                .strafeToLinearHeading(dropStackPixelPose.position, dropStackPixelPose.heading)
                .build();

        trajDropStackPixelToPark = drive.actionBuilder(dropStackPixelPose)
                .strafeToLinearHeading(beforeParkAfterDropStackPixelPose.position, beforeParkAfterDropStackPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();

        trajDropPurplePixelToAfterPurplePixelPose = drive.actionBuilder(dropPurplePixelPose)
                .strafeToLinearHeading(afterPurplePixelPose.position, afterPurplePixelPose.heading)
                .build();


        trajMidwayBackDropPoseToDropYellowPixekPose = drive.actionBuilder(midwayBackDropPose)
                .waitSeconds(3.5)
                .strafeToLinearHeading(dropYellowPixelPose.position, dropYellowPixelPose.heading)
                .build();

        trajDropYellowPixelPoseToDropStackPixelPose = drive.actionBuilder(dropYellowPixelPose)
                .strafeToLinearHeading(dropStackPixelPose.position, dropStackPixelPose.heading)
                .build();

        if (GameField.startPosition == GameField.START_POSITION.RED_RIGHT ||
                GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
            actionForRedRightBlueLeft();
        } else {
            outtakeArm.openGrip();
            actionForRedLeftBlueRight();
        }
    }

    public void actionForRedRightBlueLeft() {
        Actions.runBlocking(
                new SequentialAction(
                        trajInitToDropPurplePixel,
                        trajDropPurplePixelToDropYellowPixel
                )
        );
        telemetry.addLine("After first Action");
        telemetry.update();

        outtakeController.moveTransferToReadyForTransfer();
        safeWaitMilliSeconds(200);
        outtakeController.moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST);
        safeWaitMilliSeconds( 800);
        outtakeController.dropOnePixel();
        safeWaitMilliSeconds(500);

        telemetry.addLine("After second Action");
        telemetry.update();

        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    //trajDropYellowPixelToPark
                    drive.actionBuilder(dropYellowPixelPose)
                            .strafeToLinearHeading(parkPose.position, parkPose.heading)
                            .build()
            );

            printDebugMessages();
        } else { // FULL AUTONOMOUS
            outtakeController.moveDropToTravel();
            safeWaitMilliSeconds(500);

            printDebugMessages();
            Actions.runBlocking(
                    new SequentialAction(
                            trajDropYellowPixelPoseToMidwayBackDropPose,
                            trajMidwayBackDropPoseToMidwayStackPose,
                            trajAfterPurplePixelPoseToStackPose
                    )
            );

            printDebugMessages();
            intakeAtStack(2);
            safeWaitMilliSeconds(200);

            outtakeController.outtakeArm.openGrip();
            outtakeController.moveTravelToReadyForTransfer();
            safeWaitMilliSeconds(300);
            outtakeController.moveReadyForTransferToTransfer();
            safeWaitMilliSeconds(200);
            printDebugMessages();
            Actions.runBlocking(
                    new SequentialAction(
                            trajStackPoseToMidwayStackPose,
                            trajMidwayStackPoseToMidwayBackDropPose,
                            trajMidwayBackDropPoseToDropStackPixelPose
                    )
            );
            printDebugMessages();
            outtakeTransfertoDropTwoPixels();

            Actions.runBlocking(
                    trajDropStackPixelToPark
            );
            printDebugMessages();
        }

        outtakeController.moveOuttakeToEndState();
        printDebugMessages();
        safeWaitMilliSeconds(3000);

        telemetry.addLine("After third Action");
        telemetry.update();
    }

    public void actionForRedLeftBlueRight(){
        Actions.runBlocking(
                new SequentialAction(
                        trajInitToDropPurplePixel,
                        trajDropPurplePixelToAfterPurplePixelPose,
                        trajAfterPurplePixelPoseToStackPose
                )
        );
        /*
        outtakeController.moveTransferToReadyForTransferAuto();
        safeWaitMilliSeconds(200);
        intakeAtStack(1);
        safeWaitMilliSeconds(200);
        outtakeController.moveReadyForTransferToTransfer();
        safeWaitMilliSeconds(200);
         */

        Actions.runBlocking(
                new SequentialAction(
                        trajStackPoseToMidwayStackPose,
                        trajMidwayStackPoseToMidwayBackDropPose,
                        trajMidwayBackDropPoseToDropYellowPixekPose
                )
        );

        //outtakeController.moveTransferToReadyForTransfer();
        //safeWaitMilliSeconds(300);
        outtakeTransfertoDropTwoPixels();


        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    trajDropYellowPixelToPark
            );

            printDebugMessages();
        } else { // FULL AUTONOMOUS
            outtakeController.moveDropToTravel();

            printDebugMessages();
            Actions.runBlocking(
                    new SequentialAction(
                            trajDropYellowPixelPoseToMidwayBackDropPose,
                            trajMidwayBackDropPoseToMidwayStackPose,
                            trajAfterPurplePixelPoseToStackPose
                    )
            );

            printDebugMessages();

            intakeAtStack(2);
            safeWaitMilliSeconds(200);
            outtakeController.moveTravelToReadyForTransfer();
            safeWaitMilliSeconds(300);
            outtakeController.moveReadyForTransferToTransfer();
            safeWaitMilliSeconds(200);
            printDebugMessages();
            Actions.runBlocking(
                    new SequentialAction(
                            trajStackPoseToMidwayStackPose,
                            trajMidwayStackPoseToMidwayBackDropPose,
                            trajMidwayBackDropPoseToDropStackPixelPose
                    )
            );
            printDebugMessages();
            outtakeTransfertoDropTwoPixels();

            Actions.runBlocking(
                    trajDropStackPixelToPark
            );
            printDebugMessages();
        }

        outtakeController.moveOuttakeToEndState();
        printDebugMessages();
        safeWaitMilliSeconds(3000);

        telemetry.addLine("After third Action");
        telemetry.update();

    }

    public void rotateDriveCorrectly(Pose2d currentPose){
        double correctionAngle = Math.toDegrees(drive.pose.heading.log()) - Math.toDegrees(currentPose.heading.log());
        Actions.runBlocking(
                drive.actionBuilder(currentPose)
                        .turn(Math.toRadians(correctionAngle))
                        .build()
        );
    }


    public int stackCounter = 5;

    public void intakeAtStack(int count){
        intake.moveIntakeRollerToLevel(stackCounter + 1);
        intake.startIntakeInward();
        safeWaitMilliSeconds(200);
        intake.moveIntakeRollerToLevel(stackCounter--);
        safeWaitMilliSeconds(400);
        if (count >= 2) {
            intake.moveIntakeRollerToLevel(stackCounter--);
            safeWaitMilliSeconds(400);
        }
        intake.reverseIntake();
        safeWaitMilliSeconds(400);
        intake.stopIntakeMotor();
        intake.moveIntakeRollerToLevel(7);

    }

    public Action intakeAtStackAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                intakeAtStack(2);
                return true;
            }
        };
    }

    public Action outtakeTravelToReadyForTransfer(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                if (autoOption == AUTO_OPTION.FULL_AUTONOMOUS) {
                    outtakeController.moveTravelToReadyForTransfer();
                }
                return true;
            }
        };
    }

    public Action outtakeReadyForTransferToTransfer(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                if (autoOption == AUTO_OPTION.FULL_AUTONOMOUS) {
                    outtakeController.moveReadyForTransferToTransfer();
                }
                return true;
            }
        };
    }

    public void outtakeTransfertoDropTwoPixels() {
        outtakeController.moveTransferToPickup();
        safeWaitMilliSeconds(50);
        outtakeController.movePickupToTransfer();
        safeWaitMilliSeconds(50);
        outtakeController.moveTransferToReadyForTransfer();
        safeWaitMilliSeconds(500);
        outtakeController.moveReadyForTransferToDropLevel(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE);
        safeWaitMilliSeconds(800);
        outtakeArm.autoDropOnePixel();
        safeWaitMilliSeconds(600); //300
        outtakeArm.autoDropOnePixel();
        safeWaitMilliSeconds(500);
    }
    public Action outtakeTransfertoDropPixel(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                outtakeTransfertoDropTwoPixels();
                return true;
            }
        };
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        //telemetry.setAutoClear(true);
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
                GameField.startPosition = GameField.START_POSITION.BLUE_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                GameField.startPosition = GameField.START_POSITION.BLUE_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                GameField.startPosition = GameField.START_POSITION.RED_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetButtonAPress()){
                GameField.startPosition = GameField.START_POSITION.RED_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            telemetry.update();
        }

        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
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
        /*
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
        }*/

        if (autoOption == AUTO_OPTION.FULL_AUTONOMOUS) {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addLine("Select Pathway and Parking option");
                telemetry.addData("    Through the wall Rigging ", "Y / Δ");
                telemetry.addData("    Through the Stage Door", "B / O ");
                if (gamepadController.gp1GetButtonYPress()) {
                    pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
                    parkingOption = PARKING_OPTION.RIGGING_WALL;
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                    parkingOption = PARKING_OPTION.STAGEDOOR;
                    break;
                }
                telemetry.update();
            }
        } else {

            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addData("Selected Pathway option", pathwayOption);
                telemetry.addLine("Select Parking Position:");
                telemetry.addData("    Near wall Rigging ", "Y / Δ");
                telemetry.addData("    Near Stage Door", "B / O ");
                if (gamepadController.gp1GetButtonYPress()) {
                    pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
                    parkingOption = PARKING_OPTION.RIGGING_WALL;
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                    parkingOption = PARKING_OPTION.STAGEDOOR;
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

        /* Create VisionSensor */
        visionSensor = new VisionSensor(hardwareMap, telemetry);
        telemetry.addLine("Vision Sensor Initialized");
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
                outtakeSlides, outtakeArm, climber, launcher, visionSensor, lights, telemetry, this);
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

    public String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
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
            telemetry.addLine("Running Hazmat Autonomous Mode");
            telemetry.addData("Game Timer : ", gameTimer.time());
            telemetry.addData("PoseEstimateString :", toStringPose2d(drive.pose));
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);

            driveTrain.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }


}   // end class
