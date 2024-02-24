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

package org.firstinspires.ftc.teamcode.GameOpModes.OldAuto;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.GameOpModes.OldAuto.GameField;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.VisionOpenCV;
import org.firstinspires.ftc.teamcode.SubSystems.VisionSensor;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "HazmatAutonomous Mode GBHACKERZ4", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class AutoGBHackerz4 extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public VisionSensor visionSensor;
    //public VisionTfod visionTfodFront;
    public VisionOpenCV visionOpenCV;
    public Lights lights;
    public OuttakeController outtakeController;
    public IntakeController intakeController;
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
        //visionTfodFront.initTfod();
        visionOpenCV.initOpenCV();

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
            //visionTfodFront.runTfodTensorFlow();
            visionOpenCV.runOpenCVObjectDetection();
            telemetry.addData("Vision identified SpikeMark Location", visionOpenCV.identifiedSpikeMarkLocation);
            telemetry.addData("Pixel Loops count", pixelLoopCount);
            telemetry.update();
        }

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
    Pose2d dropPurplePixelPoseWall = new Pose2d(0,0,0);
    Pose2d afterPurplePixelPose = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseWall = new Pose2d(0, 0, 0);
    Pose2d stageDoorStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayTrussPose = new Pose2d(0,0,0);
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

    Action trajInitToDropPurplePixel, trajDropPurplePixelTodropYellowPixel, trajDropYellowPixelToPark;
    Action trajInitToDropYellowPixel, trajDropYellowPixelToDropPurplePixel, trajDropPurplePixelToPark;
    Action trajDropYellowPixelToStack;
    Action trajStackToDropStackPixel, trajDropStackPixelToStack, trajDropStackPixelToPark;

    Action trajDropPurplePixelToStack, trajStackToDropYellowPixel;


    public void runAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        moveBeyondTrussPose = new Pose2d(9, 0, 0); //x15

        //outtakeArm.backPlateAlignDown();

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionOpenCV.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(34, 15.7, Math.toRadians(-90)); //x22.2, 3.3, 25
                        dropPurplePixelPoseWall = new Pose2d(22.7, 3.0, Math.toRadians(25));
                        dropYellowPixelPose = new Pose2d(23.5, 33.5, Math.toRadians(-90));//x16.7, y32.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(18.7,30, Math.toRadians(-90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(38.3, 9, Math.toRadians(-90)); //x28, y-1.6, 10.7
                        dropPurplePixelPoseWall = new Pose2d(28, -1.6, Math.toRadians(10.7));
                        dropYellowPixelPose = new Pose2d(27.5, 33.5,  Math.toRadians(-90));//x25, y35
                        beforeParkAfterDropYellowPixelPose = new Pose2d(25,30, Math.toRadians(-90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(27.8, -4.5, Math.toRadians(-90)); //22.7,-6.7,-60//x24.5, y-9, -36.7
                        dropPurplePixelPoseWall = new Pose2d(21.5, -8, Math.toRadians(-50));
                        dropYellowPixelPose = new Pose2d(35, 33.5, Math.toRadians(-90)); //y=33.5, x35
                        beforeParkAfterDropYellowPixelPose = new Pose2d(33.5,30, Math.toRadians(-90));
                        break;
                }
                //afterPurplePixelPose = new Pose2d(8,-14,Math.toRadians(-30));
                wallStackPose = new Pose2d(13, -56, Math.toRadians(-90)); //-60
                wallMidwayStackPose = new Pose2d(2, -52, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(2, 18, Math.toRadians(-90));
                //stageDoorStackPose = new Pose2d(49, -60, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(55, -76, Math.toRadians(-90));//x51, y-73.8, -90
                //stageMidwayStackPose = new Pose2d(49, -56, Math.toRadians(-90));
                stageMidwayTrussPose = new Pose2d(50, -29, Math.toRadians(-90));//x53
                stageMidwayBackDropPose = new Pose2d(54, 18, Math.toRadians(-90)); //x53
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(18.7, 32.5, Math.toRadians(-90));//x20, y35.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(18.7, 30, Math.toRadians(-90));//x20, y35.5
                } else {
                    dropStackPixelPose = new Pose2d(26, 32, Math.toRadians(-90)); //y=32.5, x33.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(31.5, 30, Math.toRadians(-90)); //y=33.5, x35.5
                }

                parkPoseWall = new Pose2d(2.5, 27, Math.toRadians(-90)); //x2, x26
                parkPoseStageDoor = new Pose2d(48, 32, Math.toRadians(-90)); //x50, y30

                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionOpenCV.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(25.5, 1.6, Math.toRadians(90));//x20.5, y8,51
                        dropPurplePixelPoseWall = new Pose2d(21.2, 9.5, Math.toRadians(49));
                        dropYellowPixelPose = new Pose2d(33.8, -36, Math.toRadians(90)); //x=34.5,y=-35.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(33,-30, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(36, -13, Math.toRadians(90));//x27.6, y0, 13
                        dropPurplePixelPoseWall = new Pose2d(27.9, 2, Math.toRadians(18.5));
                        dropYellowPixelPose = new Pose2d(28, -36,  Math.toRadians(90));//x28.6, y-35.5
                        beforeParkAfterDropYellowPixelPose = new Pose2d(26,-30, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(35, -20, Math.toRadians(90));//x21, y-1.5, -24
                        dropPurplePixelPoseWall = new Pose2d(20.6, -2.8, Math.toRadians(-22));
                        dropYellowPixelPose = new Pose2d(22, -36, Math.toRadians(90));//x18, y-35
                        beforeParkAfterDropYellowPixelPose = new Pose2d(16.5,-30, Math.toRadians(90));
                        break;
                }
                afterPurplePixelPose = new Pose2d(8,14,Math.toRadians(30));
                wallStackPose = new Pose2d(13, 69, Math.toRadians(90));//73
                wallMidwayStackPose = new Pose2d(2, 52, Math.toRadians(90));
                wallMidwayBackDropPose = new Pose2d(2, -18, Math.toRadians(90));
                stageDoorStackPose = new Pose2d(43.5, 73, Math.toRadians(90));//x55.2, y72.2, 90
                //stageMidwayStackPose = new Pose2d(52, 56, Math.toRadians(90));//x47
                stageMidwayTrussPose = new Pose2d(52,27.5,Math.toRadians(90));
                stageMidwayBackDropPose = new Pose2d(53, -21, Math.toRadians(90));//x47
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                    beforeParkAfterDropStackPixelPose = new Pose2d(37,-32,Math.toRadians(90));
                } else {
                    dropStackPixelPose = new Pose2d(30, -35, Math.toRadians(90));//x20, -34.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(21,-32,Math.toRadians(90));
                }

                parkPoseWall = new Pose2d(3, -28, Math.toRadians(90));
                parkPoseStageDoor = new Pose2d(49, -28, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionOpenCV.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(22.2, 8.8, Math.toRadians(47.8));//23.5, 9, 62
                        dropPurplePixelPoseWall = new Pose2d(25, 1.5, Math.toRadians(90));
                        dropYellowPixelPose = new Pose2d(27, 85.9, Math.toRadians(-90));//x18, y87
                        beforeParkAfterDropYellowPixelPose = new Pose2d(16.5,82, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(7.3,-4, Math.toRadians(34)); //x17, y-4,-90
                        //stageMidwayStackPose = new Pose2d(49.7, -7, Math.toRadians(-90)); //x43, y8
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(30.5, 5.4, Math.toRadians(19)); //x26.4, y7.8, 2
                        dropPurplePixelPoseWall = new Pose2d(23.7, -2, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(33, 85.5,  Math.toRadians(-90)); //x24.6, y88
                        beforeParkAfterDropYellowPixelPose = new Pose2d(22.9,82, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(7.3,-4,Math.toRadians(18));//17, -4, -9
                        //stageMidwayStackPose = new Pose2d(51.8, -15, Math.toRadians(-90)); //x49, y9, -90
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(42.3, 1.4, Math.toRadians(-131.3)); //x25.5, y-2.5, -52.5
                        dropPurplePixelPoseWall = new Pose2d(13.5,-6.6,Math.toRadians(0));//22.5, -1.8, -46
                        dropYellowPixelPose = new Pose2d(40, 85.5, Math.toRadians(-90)); //x30.5, 88
                        beforeParkAfterDropYellowPixelPose = new Pose2d(25.2,82, Math.toRadians(-90));
                        afterPurplePixelPose = new Pose2d(46,3.4,Math.toRadians(-90)); //x15, y5, -90degrees
                        //stageMidwayStackPose = new Pose2d(53, 7, Math.toRadians(-90)); //x44, y5, 0
                        break;
                }
                afterPurplePixelPoseWall = new Pose2d(3.5,0,Math.toRadians(-90)); //x15, y5, -90degrees
                wallStackPose = new Pose2d(27, -14, Math.toRadians(-90)); //x26,-18
                wallMidwayStackPose = new Pose2d(2, 5, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(4, 72, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(52, -15, Math.toRadians(-90));//x53, y-19
                stageMidwayTrussPose = new Pose2d(54, 26.8, Math.toRadians(-90));
                stageMidwayBackDropPose = new Pose2d(54, 71, Math.toRadians(-90)); //x52, y73
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(35.5, 86.5,  Math.toRadians(-90));//x20, y35.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(18.7, 30, Math.toRadians(-90));//x20, y35.5
                } else {
                    dropStackPixelPose = new Pose2d(31.5, 34.5, Math.toRadians(-90)); //y=33.5, x35.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(31.5, 30, Math.toRadians(-90)); //y=33.5, x35.5
                }

                parkPoseWall = new Pose2d(6.5, 82.8, Math.toRadians(-90));//x3, y80.8
                parkPoseStageDoor = new Pose2d(46, 83.5, Math.toRadians(-90)); //x54.5, y77
                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                switch (visionOpenCV.identifiedSpikeMarkLocation) {
                    case LEFT:
                        dropPurplePixelPose = new Pose2d(37, 0, Math.toRadians(126)); //x20.8, y3.3, 31.5
                        dropPurplePixelPoseWall = new Pose2d(14.5, 9, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(40, -88.5, Math.toRadians(90));//x32, y-91
                        beforeParkAfterDropYellowPixelPose = new Pose2d(34,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(48,-4,Math.toRadians(89));
                        //stageMidwayStackPose = new Pose2d(43, 8, Math.toRadians(90));
                        break;
                    case MIDDLE:
                        dropPurplePixelPose = new Pose2d(28.7, -4.3, Math.toRadians(-13.8)); //x28.1, y-4, -20
                        dropPurplePixelPoseWall = new Pose2d(23.5, 3.6, Math.toRadians(0));
                        dropYellowPixelPose = new Pose2d(30.5, -88.8,  Math.toRadians(90)); //x28,y=-91
                        beforeParkAfterDropYellowPixelPose = new Pose2d(30,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(18,4,Math.toRadians(-30));//x18,y4,-30
                        //stageMidwayStackPose = new Pose2d(49, 9, Math.toRadians(90));
                        break;
                    case RIGHT:
                        dropPurplePixelPose = new Pose2d(22.3, -7.5, Math.toRadians(-60.3)); //x25.1, y-5.3, -71
                        dropPurplePixelPoseWall = new Pose2d(26.6, -1.9, Math.toRadians(-90));
                        dropYellowPixelPose = new Pose2d(26, -89, Math.toRadians(90));//x23 y=-88
                        beforeParkAfterDropYellowPixelPose = new Pose2d(25,-85, Math.toRadians(90));
                        afterPurplePixelPose = new Pose2d(18,4,Math.toRadians(-30));//x18,y4,-30
                        //stageMidwayStackPose = new Pose2d(49, 9, Math.toRadians(90));
                        break;
                }
                afterPurplePixelPoseWall = new Pose2d(3.5,3.1,Math.toRadians(90));//x18,y4,-30
                wallStackPose = new Pose2d(24.6, 15.5, Math.toRadians(90)); //x22, x19.6, 90
                wallMidwayStackPose = new Pose2d(2, 0, Math.toRadians(90));//x2, y-5
                wallMidwayBackDropPose = new Pose2d(3.7, -66, Math.toRadians(90));//x2, y-73
                stageDoorStackPose = new Pose2d(49.2, 16, Math.toRadians(90));//x48.5, y19.7
                stageMidwayTrussPose = new Pose2d(50.4,-20.6,90);//x48.5, y27.5, 90
                stageMidwayBackDropPose = new Pose2d(50.4, -72, Math.toRadians(90));//x49
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(36.1, -88,  Math.toRadians(90));
                    beforeParkAfterDropStackPixelPose = new Pose2d(36,-85, Math.toRadians(90));
                } else {
                    dropStackPixelPose = new Pose2d(26, -89, Math.toRadians(90));//x16,-89
                    beforeParkAfterDropStackPixelPose = new Pose2d(36,-85, Math.toRadians(90));
                }

                parkPoseWall = new Pose2d(4, -86, Math.toRadians(90)); //x-1, y-81
                parkPoseStageDoor = new Pose2d(48.8, -83, Math.toRadians(90));//x46, y-79
                break;
        }

        if (parkingOption == PARKING_OPTION.RIGGING_WALL) {
            parkPose = parkPoseWall;
        } else {
            parkPose = parkPoseStageDoor;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();


        //For RED_RIGHT & BLUE_LEFT
        trajInitToDropYellowPixel = drive.actionBuilder(initPose)
                .splineToLinearHeading(dropYellowPixelPose,0)
                .build();

        trajDropYellowPixelToDropPurplePixel = drive.actionBuilder(dropYellowPixelPose)
                .strafeToLinearHeading(dropPurplePixelPose.position, dropPurplePixelPose.heading)
                .build();

        trajDropPurplePixelToPark = drive.actionBuilder(dropPurplePixelPose)
                .setReversed(true)
                .splineToLinearHeading(parkPose, 0)
                .build();

        //For RED_LEFT & BLUE_RIGHT
        trajDropYellowPixelToPark = drive.actionBuilder(dropYellowPixelPose)
                .strafeToLinearHeading(beforeParkAfterDropYellowPixelPose.position, beforeParkAfterDropYellowPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
                .build();

        if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
            trajInitToDropPurplePixel = drive.actionBuilder(drive.pose)
                    //.splineTo(dropPurplePixelPose.position,dropPurplePixelPose.heading)
                    .splineToLinearHeading(dropPurplePixelPose,0)
                    .build();
            //FOR BLUE_RIGHT & RED_LEFT
            trajDropPurplePixelToStack = drive.actionBuilder(dropPurplePixelPose)
                    .strafeToLinearHeading(afterPurplePixelPose.position, afterPurplePixelPose.heading)
                    //.strafeToLinearHeading(midwayStackPose.position, midwayStackPose.heading)
                    .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                    .build();
            trajStackToDropYellowPixel = drive.actionBuilder(stageDoorStackPose)
                    .setReversed(true)
                    //.strafeTo(stageMidwayTrussPose.position)
                    .strafeTo(stageMidwayBackDropPose.position)
                    .splineToLinearHeading(dropYellowPixelPose, 0)
                    .build();

            //FOR BLUE_LEFT & RED_RIGHT
            trajDropPurplePixelToStack = drive.actionBuilder(dropPurplePixelPose)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position, stageMidwayBackDropPose.heading)
                    //.strafeTo(stageMidwayStackPose.position)
                    .strafeTo(stageDoorStackPose.position)
                    .build();


            trajStackToDropStackPixel = drive.actionBuilder(stageDoorStackPose)
                    //.splineTo(stageMidwayBackDropPose.position, stageMidwayBackDropPose.heading)
                    //.splineTo(dropStackPixelPose.position, dropStackPixelPose.heading)
                    //.splineToLinearHeading(stageMidwayBackDropPose, 0)
                    //.strafeTo(stageMidwayTrussPose.position)
                    .strafeTo(stageMidwayBackDropPose.position)
                    .setReversed(true)
                    .splineToLinearHeading(dropStackPixelPose, 0)
                    .build();

            trajDropStackPixelToStack = drive.actionBuilder(dropStackPixelPose)
                    //.splineTo(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading)
                    //.splineTo(stageDoorStackPose.position, stageDoorStackPose.heading)
                    .splineToLinearHeading(stageMidwayBackDropPose,0)
                    .strafeTo(stageDoorStackPose.position)
                    //.splineToLinearHeading(stageDoorStackPose, 0)
                    .build();
        } else {
            //FOR BLUE_RIGHT & RED_LEFT
            trajInitToDropPurplePixel = drive.actionBuilder(drive.pose)
                    //.splineTo(dropPurplePixelPose.position,dropPurplePixelPose.heading)
                    .splineToLinearHeading(dropPurplePixelPoseWall,0)
                    .build();
            trajDropPurplePixelToStack = drive.actionBuilder(dropPurplePixelPoseWall)
                    .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading)
                    //.strafeToLinearHeading(midwayStackPose.position, midwayStackPose.heading)
                    //.strafeToLinearHeading(wallStackPose.position, wallStackPose.heading)
                    .build();
            trajStackToDropYellowPixel = drive.actionBuilder(afterPurplePixelPoseWall)
                    .splineToLinearHeading(wallMidwayStackPose,0)
                    .strafeTo(wallMidwayBackDropPose.position)
                    .splineToLinearHeading(dropYellowPixelPose, 0)
                    .build();
            trajDropPurplePixelTodropYellowPixel = drive.actionBuilder(dropPurplePixelPoseWall)
                    .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                    .strafeTo(wallMidwayBackDropPose.position)
                    .splineToLinearHeading(dropYellowPixelPose,0)
                    .build();

            //FOR BLUE_LEFT & RED_RIGHT
            trajDropYellowPixelToStack = drive.actionBuilder(dropYellowPixelPose)
                    .splineToLinearHeading(wallMidwayBackDropPose, 0)
                    //.splineToLinearHeading(wallMidwayStackPose,0)
                    .strafeTo(wallMidwayStackPose.position)
                    .splineToLinearHeading(wallStackPose, 0)
                    .build();
            trajStackToDropStackPixel = drive.actionBuilder(wallStackPose)
                    .setReversed(true)
                    .splineToLinearHeading(wallMidwayStackPose, 0)
                    //.splineToLinearHeading(wallMidwayBackDropPose,0)
                    .strafeTo(wallMidwayStackPose.position)
                    .splineToLinearHeading(dropStackPixelPose, 0)
                    .build();
            trajDropStackPixelToStack = drive.actionBuilder(dropStackPixelPose)
                    .splineToLinearHeading(wallMidwayBackDropPose,0)
                    //.splineToLinearHeading(wallMidwayStackPose,0)
                    .strafeTo(wallMidwayStackPose.position)
                    .splineToLinearHeading(wallStackPose, 0)
                    .build();
        }

        trajDropStackPixelToPark = drive.actionBuilder(dropStackPixelPose)
                .strafeToLinearHeading(beforeParkAfterDropStackPixelPose.position, beforeParkAfterDropStackPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading)
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
                        new ParallelAction(
                                trajInitToDropYellowPixel,
                                new SequentialAction(
                                        outtakeController.moveTransferToReadyForTransferAction(),
                                        new SleepAction(0.3),
                                        outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST)
                                ),
                                intakeController.dropLiftIntake()
                        ),
                        outtakeController.dropOnePixelAction(),
                        new SleepAction(0.1),
                        /*
                        outtakeController.dropOnePixelAction(),
                        new SleepAction(0.2),
                         */
                        trajDropYellowPixelToDropPurplePixel,
                        intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                        intakeController.dropPurplePixelUsingIntakeAction()
                )
        );

        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    new SleepAction(0.2),
                                    intakeController.intakeLiftUpAction(),
                                    trajDropPurplePixelToPark
                                    ),
                            outtakeController.moveOuttakeToEndStateAction()
                    )
            );
        } else { //FULL AUTO
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    new SequentialAction(
                                            new SleepAction(0.2),
                                            intakeController.intakeLiftUpAction()
                                    ),
                                    outtakeController.moveDropToReadyforTransferAction()
                            ),
                            new ParallelAction(
                                    trajDropPurplePixelToStack,
                                    new SequentialAction(
                                            new SleepAction(0.3),
                                            intakeController.dropLiftIntake()
                                    )
                            ),
                            intakeController.intakeAtStackTwoPixelsAction(),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    trajStackToDropStackPixel,
                                    new SequentialAction(
                                            intakeController.intakeLiftUpAction(),
                                            outtakeController.moveReadyForTransferToTransferAction(),
                                            new SleepAction(0.1),
                                            outtakeController.moveTransferToPickupAction(),
                                            new SleepAction(0.2),
                                            outtakeController.moveTransferToReadyForTransferAction()
                                    )
                            ),
                            /*
                            outtakeController.moveTransferToPickupAction(),
                            new SleepAction(0.2),
                            outtakeController.moveTransferToReadyForTransferAction(),
                             */
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE),
                            new SleepAction(0.75),
                            outtakeController.dropOnePixelAction(),
                            new SleepAction(0.25),
                            outtakeController.dropOnePixelAction()
                    )
                );

            if(pixelLoopCount == 2) {
                Actions.runBlocking(
                        new SequentialAction(
                                new SequentialAction(
                                        new ParallelAction(
                                                outtakeController.moveDropToReadyforTransferAction(),
                                                trajDropStackPixelToStack
                                        ),
                                        intakeController.intakeAtStackTwoPixelsAction(),
                                        new SleepAction(1),
                                        intakeController.intakeLiftUpAction()
                                ),
                                new ParallelAction(
                                        trajStackToDropStackPixel,
                                        new SequentialAction(
                                                outtakeController.moveReadyForTransferToTransferAction(),
                                                outtakeController.moveTransferToReadyForTransferAction()
                                        )
                                ),
                                outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID),
                                outtakeController.dropOnePixelAction(),
                                new SleepAction(0.2),
                                outtakeController.dropOnePixelAction(),
                                new SleepAction(0.2)
                        )
                );
            }
            Actions.runBlocking(
                    outtakeController.moveOuttakeToEndStateAction()
            );
        }
    }

    public void actionForRedLeftBlueRight(){
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                trajInitToDropPurplePixel,
                                intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                intakeController.dropLiftIntake()
                        ),
                        new SleepAction(0.5),
                        intakeController.dropPurplePixelUsingIntakeAction(),
                        outtakeController.moveDropToReadyforTransferAction(),
                        intakeController.intakeLiftUpAction(),
                        new SleepAction(8.5),
                        trajDropPurplePixelTodropYellowPixel,
                        outtakeController.moveReadyForTransferToTransferAction(),
                        new SleepAction(0.1),
                        outtakeController.moveTransferToPickupAction(),
                        new SleepAction(0.3),
                        outtakeController.moveTransferToReadyForTransferAction(),
                        outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST),
                        new SleepAction(0.75),
                        outtakeController.dropOnePixelAction(),
                        new SleepAction(0.15),
                        outtakeController.dropOnePixelAction()
                )
        );

        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    new ParallelAction(
                            trajDropYellowPixelToPark,
                            outtakeController.moveOuttakeToEndStateAction()
                    )
            );
        } else { //FULL AUTO
            Actions.runBlocking(
                    new SequentialAction(
                            new SequentialAction(
                                    new ParallelAction(
                                            outtakeController.moveDropToReadyforTransferAction(),
                                            trajDropYellowPixelToStack
                                    ),
                                    intakeController.intakeAtStackTwoPixelsAction(),
                                    new SleepAction(1),
                                    intakeController.intakeLiftUpAction()
                            ),
                            new ParallelAction(
                                    trajStackToDropStackPixel,
                                    new SequentialAction(
                                            outtakeController.moveReadyForTransferToTransferAction(),
                                            outtakeController.moveTransferToReadyForTransferAction()
                                    )
                            ),
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID),
                            outtakeController.dropOnePixelAction(),
                            outtakeController.dropOnePixelAction()
                    )
            );

            if(pixelLoopCount == 2) {
                Actions.runBlocking(
                        new SequentialAction(
                                new SequentialAction(
                                        new ParallelAction(
                                                outtakeController.moveDropToReadyforTransferAction(),
                                                trajDropStackPixelToStack
                                        ),
                                        intakeController.intakeAtStackTwoPixelsAction(),
                                        new SleepAction(1),
                                        intakeController.intakeLiftUpAction()
                                ),
                                new ParallelAction(
                                        trajStackToDropStackPixel,
                                        new SequentialAction(
                                                outtakeController.moveReadyForTransferToTransferAction(),
                                                outtakeController.moveTransferToReadyForTransferAction()
                                        )
                                ),
                                outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_BELOW_MID),
                                outtakeController.dropOnePixelAction(),
                                outtakeController.dropOnePixelAction()
                        )
                );
            }
            Actions.runBlocking(
                    outtakeController.moveOuttakeToEndStateAction()
            );
        }
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
            if(gamepadController.gp1GetSquarePress()){
                GameField.startPosition = GameField.START_POSITION.BLUE_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetTrianglePress()){
                GameField.startPosition = GameField.START_POSITION.BLUE_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetCirclePress()){
                GameField.startPosition = GameField.START_POSITION.RED_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetCrossPress()){
                GameField.startPosition = GameField.START_POSITION.RED_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            telemetry.update();
        }

        if (GameField.startPosition == GameField.START_POSITION.BLUE_RIGHT ||
                GameField.startPosition == GameField.START_POSITION.RED_LEFT) {
            autoOption = AUTO_OPTION.PRELOAD_AND_PARK;
        } else {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addLine("Select Auto Options");
                telemetry.addData("    Drop Preload and Park       ", "(X / ▢)");
                telemetry.addData("    Full autonomous (1 cycle)   ", "(Y / Δ)");
                telemetry.addData("    Full autonomous (2 cycle)   ","(B / O)");

                if (gamepadController.gp1GetSquarePress()) {
                    autoOption = AUTO_OPTION.PRELOAD_AND_PARK;
                    break;
                }
                if (gamepadController.gp1GetTrianglePress()) {
                    autoOption = AUTO_OPTION.FULL_AUTONOMOUS;
                    pixelLoopCount = 1;
                    break;
                }
                if(gamepadController.gp1GetCirclePress()){
                    autoOption = AUTO_OPTION.FULL_AUTONOMOUS;
                    pixelLoopCount = 2;
                    break;
                }
                telemetry.update();
            }
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addLine("Select Pathway and Parking option");
            telemetry.addData("    Wall Rigging ", "Y / Δ");
            telemetry.addData("    Stage Door", "B / O ");
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
        //visionTfodFront = new VisionTfod(hardwareMap, telemetry, "Webcam 1");
        //telemetry.addLine("VisionTfod Initialized");
        visionOpenCV = new VisionOpenCV(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("VisionOpenCV Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        outtakeController = new OuttakeController(this.outtakeSlides, this.outtakeArm, this);
        telemetry.addLine("Outtake Controller Initialized");
        telemetry.update();

        intakeController = new IntakeController(this.intake, this.magazine, this);
        telemetry.addLine("Intake Controller Initialized");
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

            driveTrain.printDebugMessages();
            lights.printDebugMessages();
        }
        telemetry.update();
    }
}   // end class
