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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
import org.firstinspires.ftc.teamcode.Controllers.OuttakeController;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
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
@Autonomous(name = "HazmatAuto State 2", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
@Disabled
public class AutoState2 extends LinearOpMode {

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

        //Build trajectories
        buildAutonoumousMode();

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

            //Build parking trajectory based on last detected target by vision
            setTrajectoryBasedOnVision();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEFAULT);

            if (GameField.startPosition == GameField.START_POSITION.RED_RIGHT ||
                    GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
                runActionForRedRightBlueLeft();
            } else {
                outtakeArm.openGrip();
                runActionForRedLeftBlueRight();
            }
        }
    }   // end runOpMode()

    Pose2d initPose= new Pose2d(0, 0, 0);

    Pose2d dropPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d dropPurplePixelPoseWallLeft = new Pose2d(0,0,0);
    Pose2d dropPurplePixelPoseWallMiddle = new Pose2d(0,0,0);
    Pose2d dropPurplePixelPoseWallRight = new Pose2d(0,0,0);

    Pose2d afterPurplePixelPose = new Pose2d(0, 0, 0); //TODO: FIX BUG
    Pose2d afterPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d afterPurplePixelPoseWall = new Pose2d(0, 0, 0);//TODO: CHECK

    Pose2d stageDoorStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayTrussPose = new Pose2d(0,0,0);
    Pose2d stageMidwayBackDropPose = new Pose2d(0, 0, 0);
    Pose2d wallStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayBackDropPose = new Pose2d(0, 0, 0);

    Pose2d dropYellowPixelPoseLeft= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseMiddle= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseRight= new Pose2d(0, 0, 0);

    Pose2d beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(0,0,0);
    Pose2d beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(0,0,0);
    Pose2d beforeParkAfterDropYellowPixelPoseRight = new Pose2d(0,0,0);

    Pose2d dropStackPixelPose = new Pose2d(0, 0, 0);
    Pose2d beforeParkAfterDropStackPixelPose = new Pose2d(0,0,0);
    Pose2d parkPoseStageDoor = new Pose2d(0, 0, 0);
    Pose2d parkPoseWall = new Pose2d(0, 0, 0);
    Pose2d parkPose = new Pose2d(0, 0, 0);

    Action trajInitToDropYellowPixel, trajInitToDropYellowPixelLeft, trajInitToDropYellowPixelMiddle, trajInitToDropYellowPixelRight;
    Action trajDropYellowPixelToDropPurplePixel, trajDropYellowPixelToDropPurplePixelLeft,
            trajDropYellowPixelToDropPurplePixelMiddle, trajDropYellowPixelToDropPurplePixelRight;
    Action trajDropPurplePixelToPark, trajDropPurplePixelToParkLeft, trajDropPurplePixelToParkMiddle, trajDropPurplePixelToParkRight;
    Action trajDropPurplePixelToStack, trajDropPurplePixelToStackLeft, trajDropPurplePixelToStackMiddle, trajDropPurplePixelToStackRight;
    Action trajStackToDropStackPixel, trajDropStackPixelToStack, trajDropStackPixelToPark;

    Action trajInitToDropPurplePixel, trajInitToDropPurplePixelLeft, trajInitToDropPurplePixelMiddle, trajInitToDropPurplePixelRight;
    Action trajDropPurplePixelToDropYellowPixel, trajDropPurplePixelToDropYellowPixelLeft,
            trajDropPurplePixelToDropYellowPixelMiddle, trajDropPurplePixelToDropYellowPixelRight ;
    Action trajDropYellowPixelToStack, trajDropYellowPixelToStackLeft, trajDropYellowPixelToStackMiddle, trajDropYellowPixelToStackRight;
    Action trajStackToDropYellowPixel, trajStackToDropYellowPixelLeft, trajStackToDropYellowPixelMiddle, trajStackToDropYellowPixelRight;
    Action trajDropYellowPixelToPark, trajDropYellowPixelToParkLeft, trajDropYellowPixelToParkMiddle, trajDropYellowPixelToParkRight;


    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                dropPurplePixelPoseLeft = new Pose2d(34, 15.7, Math.toRadians(-90)); //x22.2, 3.3, 25
                dropPurplePixelPoseWallLeft = new Pose2d(22.7, 3.0, Math.toRadians(25));
                dropYellowPixelPoseLeft = new Pose2d(23.5, 33.5, Math.toRadians(-90));//x16.7, y32.5
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(18.7, 30, Math.toRadians(-90));

                dropPurplePixelPoseMiddle = new Pose2d(38.3, 9, Math.toRadians(-90)); //x28, y-1.6, 10.7
                dropPurplePixelPoseWallMiddle = new Pose2d(28, -1.6, Math.toRadians(10.7));
                dropYellowPixelPoseMiddle = new Pose2d(27.5, 33.5, Math.toRadians(-90));//x25, y35
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(25, 30, Math.toRadians(-90));

                dropPurplePixelPoseRight = new Pose2d(27.8, -4.5, Math.toRadians(-90)); //22.7,-6.7,-60//x24.5, y-9, -36.7
                dropPurplePixelPoseWallRight = new Pose2d(21.5, -8, Math.toRadians(-50));
                dropYellowPixelPoseRight = new Pose2d(35, 33.5, Math.toRadians(-90)); //y=33.5, x35
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(33.5, 30, Math.toRadians(-90));

                wallStackPose = new Pose2d(13, -56, Math.toRadians(-90)); //-60
                wallMidwayStackPose = new Pose2d(2, -52, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(2, 18, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(55, -76, Math.toRadians(-90));//x51, y-73.8, -90
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
                dropPurplePixelPoseLeft = new Pose2d(25.5, 1.6, Math.toRadians(90));//x20.5, y8,51
                dropPurplePixelPoseWallLeft = new Pose2d(21.2, 9.5, Math.toRadians(49));
                dropYellowPixelPoseLeft = new Pose2d(33.8, -35, Math.toRadians(90)); //x=34.5,y=-35.5
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(33, -30, Math.toRadians(90));

                dropPurplePixelPoseMiddle = new Pose2d(36, -13, Math.toRadians(90));//x27.6, y0, 13
                dropPurplePixelPoseWallMiddle = new Pose2d(27.9, 2, Math.toRadians(18.5));
                dropYellowPixelPoseMiddle = new Pose2d(28, -35,  Math.toRadians(90));//x28.6, y-35.5
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(26, -30, Math.toRadians(90));

                dropPurplePixelPoseRight = new Pose2d(35, -20, Math.toRadians(90));//x21, y-1.5, -24
                dropPurplePixelPoseWallRight = new Pose2d(20.6, -2.8, Math.toRadians(-22));
                dropYellowPixelPoseRight = new Pose2d(21.5, -35, Math.toRadians(90));//x18, y-35
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(16.5, -30, Math.toRadians(90));

                afterPurplePixelPose = new Pose2d(8, 14, Math.toRadians(30));
                wallStackPose = new Pose2d(13, 69, Math.toRadians(90));//73
                wallMidwayStackPose = new Pose2d(2, 52, Math.toRadians(90));
                wallMidwayBackDropPose = new Pose2d(2, -18, Math.toRadians(90));
                stageDoorStackPose = new Pose2d(45, 72, Math.toRadians(90));//x55.2, y72.2, 90
                //stageMidwayStackPose = new Pose2d(52, 56, Math.toRadians(90));//x47
                stageMidwayTrussPose = new Pose2d(52,27.5,Math.toRadians(90));
                stageMidwayBackDropPose = new Pose2d(54, -21, Math.toRadians(90));//x47
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(37, -36, Math.toRadians(90));
                    beforeParkAfterDropStackPixelPose = new Pose2d(37, -32, Math.toRadians(90));
                } else {
                    dropStackPixelPose = new Pose2d(30, -35, Math.toRadians(90));//x20, -34.5
                    beforeParkAfterDropStackPixelPose = new Pose2d(21, -32, Math.toRadians(90));
                }

                parkPoseWall = new Pose2d(3, -28, Math.toRadians(90));
                parkPoseStageDoor = new Pose2d(49, -28, Math.toRadians(90));
                break;

            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);

                dropPurplePixelPoseLeft = new Pose2d(22.2, 8.8, Math.toRadians(47.8));//23.5, 9, 62
                dropPurplePixelPoseWallLeft = new Pose2d(25, 1.5, Math.toRadians(90));
                dropYellowPixelPoseLeft = new Pose2d(27, 85.9, Math.toRadians(-90));//x18, y87
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(16.5, 82, Math.toRadians(-90));
                afterPurplePixelPoseLeft = new Pose2d(7.3, -4, Math.toRadians(34)); //x17, y-4,-90

                dropPurplePixelPoseMiddle = new Pose2d(30.5, 5.4, Math.toRadians(19)); //x26.4, y7.8, 2
                dropPurplePixelPoseWallMiddle = new Pose2d(23.7, -2, Math.toRadians(0));
                dropYellowPixelPoseMiddle = new Pose2d(33, 85.5, Math.toRadians(-90)); //x24.6, y88
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(22.9, 82, Math.toRadians(-90));
                afterPurplePixelPoseMiddle = new Pose2d(7.3, -4, Math.toRadians(18));//17, -4, -9

                dropPurplePixelPoseRight = new Pose2d(42.3, 1.4, Math.toRadians(-131.3)); //x25.5, y-2.5, -52.5
                dropPurplePixelPoseWallRight = new Pose2d(13.5, -6.6, Math.toRadians(0));//22.5, -1.8, -46
                dropYellowPixelPoseRight = new Pose2d(40, 85.5, Math.toRadians(-90)); //x30.5, 88
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(25.2, 82, Math.toRadians(-90));
                afterPurplePixelPoseRight = new Pose2d(46, 3.4, Math.toRadians(-90)); //x15, y5, -90degrees

                afterPurplePixelPoseWall = new Pose2d(3.2, 0, Math.toRadians(-90)); //x15, y5, -90degrees
                wallStackPose = new Pose2d(27, -14, Math.toRadians(-90)); //x26,-18
                wallMidwayStackPose = new Pose2d(2, 5, Math.toRadians(-90));
                wallMidwayBackDropPose = new Pose2d(11.7, 72, Math.toRadians(-90));
                stageDoorStackPose = new Pose2d(52, -15, Math.toRadians(-90));//x53, y-19
                stageMidwayTrussPose = new Pose2d(54, 26.8, Math.toRadians(-90));
                stageMidwayBackDropPose = new Pose2d(54, 71, Math.toRadians(-90)); //x52, y73
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(35.5, 86.5, Math.toRadians(-90));//x20, y35.5
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

                dropPurplePixelPoseLeft = new Pose2d(37, 0, Math.toRadians(126)); //x20.8, y3.3, 31.5
                dropPurplePixelPoseWallLeft = new Pose2d(14.5, 9, Math.toRadians(0));
                dropYellowPixelPoseLeft = new Pose2d(40, -88.5, Math.toRadians(90));//x32, y-91
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(34, -85, Math.toRadians(90));
                afterPurplePixelPoseLeft = new Pose2d(48, -4, Math.toRadians(89));

                dropPurplePixelPoseMiddle = new Pose2d(28.7, -4.3, Math.toRadians(-13.8)); //x28.1, y-4, -20
                dropPurplePixelPoseWallMiddle = new Pose2d(23.5, 3.6, Math.toRadians(0));
                dropYellowPixelPoseMiddle = new Pose2d(30.5, -88.8, Math.toRadians(90)); //x28,y=-91
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(30, -85, Math.toRadians(90));
                afterPurplePixelPoseMiddle = new Pose2d(18, 4, Math.toRadians(-30));//x18,y4,-30

                dropPurplePixelPoseRight = new Pose2d(22.3, -7.5, Math.toRadians(-60.3)); //x25.1, y-5.3, -71
                dropPurplePixelPoseWallRight = new Pose2d(26.6, -1.9, Math.toRadians(-90));
                dropYellowPixelPoseRight = new Pose2d(26, -89, Math.toRadians(90));//x23 y=-88
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(25, -85, Math.toRadians(90));
                afterPurplePixelPoseRight = new Pose2d(18, 4, Math.toRadians(-30));//x18,y4,-30

                afterPurplePixelPoseWall = new Pose2d(3.75, 3.1, Math.toRadians(90));//x18,y4,-30
                wallStackPose = new Pose2d(24.6, 15.5, Math.toRadians(90)); //x22, x19.6, 90
                wallMidwayStackPose = new Pose2d(2, 0, Math.toRadians(90));//x2, y-5
                wallMidwayBackDropPose = new Pose2d(10, -66, Math.toRadians(90));//x2, y-73
                stageDoorStackPose = new Pose2d(49.2, 16, Math.toRadians(90));//x48.5, y19.7
                stageMidwayTrussPose = new Pose2d(50.4, -20.6, 90);//x48.5, y27.5, 90
                stageMidwayBackDropPose = new Pose2d(50.4, -72, Math.toRadians(90));//x49
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                if (pathwayOption == PATHWAY_OPTION.RIGGING_WALL) {
                    dropStackPixelPose = new Pose2d(36.1, -88, Math.toRadians(90));
                    beforeParkAfterDropStackPixelPose = new Pose2d(36, -85, Math.toRadians(90));
                } else {
                    dropStackPixelPose = new Pose2d(26, -89, Math.toRadians(90));//x16,-89
                    beforeParkAfterDropStackPixelPose = new Pose2d(36, -85, Math.toRadians(90));
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
        if (GameField.startPosition == GameField.START_POSITION.RED_RIGHT ||
                GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
            trajInitToDropYellowPixelLeft = drive.actionBuilder(initPose)
                    .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                    .build();
            trajInitToDropYellowPixelMiddle = drive.actionBuilder(initPose)
                    .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                    .build();
            trajInitToDropYellowPixelRight = drive.actionBuilder(initPose)
                    .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                    .build();

            trajDropYellowPixelToDropPurplePixelLeft = drive.actionBuilder(dropYellowPixelPoseLeft)
                    .strafeToLinearHeading(dropPurplePixelPoseLeft.position, dropPurplePixelPoseLeft.heading)
                    .build();
            trajDropYellowPixelToDropPurplePixelMiddle = drive.actionBuilder(dropYellowPixelPoseMiddle)
                    .strafeToLinearHeading(dropPurplePixelPoseMiddle.position, dropPurplePixelPoseMiddle.heading)
                    .build();
            trajDropYellowPixelToDropPurplePixelRight = drive.actionBuilder(dropYellowPixelPoseRight)
                    .strafeToLinearHeading(dropPurplePixelPoseRight.position, dropPurplePixelPoseRight.heading)
                    .build();

            trajDropPurplePixelToParkLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                    .setReversed(true)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                    .setReversed(true)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkRight = drive.actionBuilder(dropPurplePixelPoseRight)
                    .setReversed(true)
                    .splineToLinearHeading(parkPose, 0)
                    .build();

            if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                trajDropPurplePixelToStackLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                        .strafeToLinearHeading(stageMidwayBackDropPose.position, stageMidwayBackDropPose.heading)
                        .strafeTo(stageDoorStackPose.position)
                        .build();
                trajDropPurplePixelToStackMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                        .strafeToLinearHeading(stageMidwayBackDropPose.position, stageMidwayBackDropPose.heading)
                        .strafeTo(stageDoorStackPose.position)
                        .build();
                trajDropPurplePixelToStackRight = drive.actionBuilder(dropPurplePixelPoseRight)
                        .strafeToLinearHeading(stageMidwayBackDropPose.position, stageMidwayBackDropPose.heading)
                        .strafeTo(stageDoorStackPose.position)
                        .build();

                trajStackToDropStackPixel = drive.actionBuilder(stageDoorStackPose)
                        .strafeTo(stageMidwayBackDropPose.position)
                        .setReversed(true)
                        .splineToLinearHeading(dropStackPixelPose, 0)
                        .build();

                trajDropStackPixelToStack = drive.actionBuilder(dropStackPixelPose)
                        .splineToLinearHeading(stageMidwayBackDropPose, 0)
                        .strafeTo(stageDoorStackPose.position)
                        .build();
            } else {
                trajDropYellowPixelToStackLeft = drive.actionBuilder(dropYellowPixelPoseLeft)
                        .splineToLinearHeading(wallMidwayBackDropPose, 0)
                        .strafeTo(wallMidwayStackPose.position)
                        .splineToLinearHeading(wallStackPose, 0)
                        .build();
                trajDropYellowPixelToStackMiddle = drive.actionBuilder(dropYellowPixelPoseMiddle)
                        .splineToLinearHeading(wallMidwayBackDropPose, 0)
                        .strafeTo(wallMidwayStackPose.position)
                        .splineToLinearHeading(wallStackPose, 0)
                        .build();
                trajDropYellowPixelToStackRight = drive.actionBuilder(dropYellowPixelPoseRight)
                        .splineToLinearHeading(wallMidwayBackDropPose, 0)
                        .strafeTo(wallMidwayStackPose.position)
                        .splineToLinearHeading(wallStackPose, 0)
                        .build();

                trajStackToDropStackPixel = drive.actionBuilder(wallStackPose)
                        .setReversed(true)
                        .splineToLinearHeading(wallMidwayStackPose, 0)
                        .strafeTo(wallMidwayStackPose.position)
                        .splineToLinearHeading(dropStackPixelPose, 0)
                        .build();
                trajDropStackPixelToStack = drive.actionBuilder(dropStackPixelPose)
                        .splineToLinearHeading(wallMidwayBackDropPose,0)
                        .strafeTo(wallMidwayStackPose.position)
                        .splineToLinearHeading(wallStackPose, 0)
                        .build();
            }

        } else {
            //For RED_LEFT & BLUE_RIGHT
            trajDropYellowPixelToParkLeft = drive.actionBuilder(dropYellowPixelPoseLeft)
                    .strafeToLinearHeading(beforeParkAfterDropYellowPixelPoseLeft.position, beforeParkAfterDropYellowPixelPoseLeft.heading)
                    .strafeToLinearHeading(parkPose.position, parkPose.heading)
                    .build();
            trajDropYellowPixelToParkMiddle = drive.actionBuilder(dropYellowPixelPoseMiddle)
                    .strafeToLinearHeading(beforeParkAfterDropYellowPixelPoseMiddle.position, beforeParkAfterDropYellowPixelPoseMiddle.heading)
                    .strafeToLinearHeading(parkPose.position, parkPose.heading)
                    .build();
            trajDropYellowPixelToParkRight = drive.actionBuilder(dropYellowPixelPoseRight)
                    .strafeToLinearHeading(beforeParkAfterDropYellowPixelPoseRight.position, beforeParkAfterDropYellowPixelPoseRight.heading)
                    .strafeToLinearHeading(parkPose.position, parkPose.heading)
                    .build();

            if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                trajInitToDropPurplePixelLeft = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseLeft, 0)
                        .build();
                trajInitToDropPurplePixelMiddle = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseMiddle, 0)
                        .build();
                trajInitToDropPurplePixelRight = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseRight, 0)
                        .build();

                //FOR BLUE_RIGHT & RED_LEFT TODO: FIX BUG
                trajDropPurplePixelToStackLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                        .strafeToLinearHeading(afterPurplePixelPoseLeft.position, afterPurplePixelPoseLeft.heading)
                        .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                        .build();
                trajDropPurplePixelToStackMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                        .strafeToLinearHeading(afterPurplePixelPoseMiddle.position, afterPurplePixelPoseMiddle.heading)
                        .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                        .build();
                trajDropPurplePixelToStackRight = drive.actionBuilder(dropPurplePixelPoseRight)
                        .strafeToLinearHeading(afterPurplePixelPoseRight.position, afterPurplePixelPoseRight.heading)
                        .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                        .build();

                trajStackToDropYellowPixelLeft = drive.actionBuilder(stageDoorStackPose)
                        .setReversed(true)
                        .strafeTo(stageMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                        .build();
                trajStackToDropYellowPixelMiddle = drive.actionBuilder(stageDoorStackPose)
                        .setReversed(true)
                        .strafeTo(stageMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                        .build();
                trajStackToDropYellowPixelRight = drive.actionBuilder(stageDoorStackPose)
                        .setReversed(true)
                        .strafeTo(stageMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                        .build();

            } else {
                //FOR BLUE_RIGHT & RED_LEFT
                trajInitToDropPurplePixelLeft = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseWallLeft, 0)
                        .build();
                trajInitToDropPurplePixelMiddle = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseWallMiddle, 0)
                        .build();
                trajInitToDropPurplePixelRight = drive.actionBuilder(drive.pose)
                        .splineToLinearHeading(dropPurplePixelPoseWallRight, 0)
                        .build();

                trajDropPurplePixelToStackLeft = drive.actionBuilder(dropPurplePixelPoseWallLeft)
                        .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading)
                        .build();
                trajDropPurplePixelToStackMiddle = drive.actionBuilder(dropPurplePixelPoseWallMiddle)
                        .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading)
                        .build();
                trajDropPurplePixelToStackRight = drive.actionBuilder(dropPurplePixelPoseWallRight)
                        .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading)
                        .build();

                trajStackToDropYellowPixelLeft = drive.actionBuilder(afterPurplePixelPoseWall)
                        .splineToLinearHeading(wallMidwayStackPose, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                        .build();
                trajStackToDropYellowPixelMiddle = drive.actionBuilder(afterPurplePixelPoseWall)
                        .splineToLinearHeading(wallMidwayStackPose, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                        .build();
                trajStackToDropYellowPixelRight = drive.actionBuilder(afterPurplePixelPoseWall)
                        .splineToLinearHeading(wallMidwayStackPose, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                        .build();

                trajDropPurplePixelToDropYellowPixelLeft = drive.actionBuilder(dropPurplePixelPoseWallLeft)
                        .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                        .build();
                trajDropPurplePixelToDropYellowPixelMiddle = drive.actionBuilder(dropPurplePixelPoseWallMiddle)
                        .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                        .build();
                trajDropPurplePixelToDropYellowPixelRight = drive.actionBuilder(dropPurplePixelPoseWallRight)
                        .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                        .strafeTo(wallMidwayBackDropPose.position)
                        .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                        .build();
            }

            //TODO: VERIFY
            trajDropStackPixelToPark = drive.actionBuilder(dropStackPixelPose)
                    .strafeToLinearHeading(beforeParkAfterDropStackPixelPose.position, beforeParkAfterDropStackPixelPose.heading)
                    .strafeToLinearHeading(parkPose.position, parkPose.heading)
                    .build();


        }
    }

    public void setTrajectoryBasedOnVision() {
        switch (visionOpenCV.identifiedSpikeMarkLocation) {
            case LEFT:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelLeft;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelLeft;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkLeft;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelLeft;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackLeft;
                trajDropPurplePixelToDropYellowPixel = trajDropPurplePixelToDropYellowPixelLeft;
                trajDropYellowPixelToStack = trajDropYellowPixelToStackLeft;
                trajStackToDropYellowPixel = trajStackToDropYellowPixelLeft;
                trajDropYellowPixelToPark = trajDropYellowPixelToParkLeft;
                break;
            case MIDDLE:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelMiddle;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelMiddle;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkMiddle;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelMiddle;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackMiddle;
                trajDropPurplePixelToDropYellowPixel = trajDropPurplePixelToDropYellowPixelMiddle;
                trajDropYellowPixelToStack = trajDropYellowPixelToStackMiddle;
                trajStackToDropYellowPixel = trajStackToDropYellowPixelMiddle;
                trajDropYellowPixelToPark = trajDropYellowPixelToParkMiddle;
                break;
            case RIGHT:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelRight;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelRight;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkRight;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelRight;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackRight;
                trajDropPurplePixelToDropYellowPixel = trajDropPurplePixelToDropYellowPixelRight;
                trajDropYellowPixelToStack = trajDropYellowPixelToStackRight;
                trajStackToDropYellowPixel = trajStackToDropYellowPixelRight;
                trajDropYellowPixelToPark = trajDropYellowPixelToParkRight;
                break;
        }
    }

    public void runActionForRedRightBlueLeft() {
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                trajInitToDropYellowPixel,
                                new SequentialAction(
                                        outtakeController.moveTransferToReadyForTransferAction(),
                                        //new SleepAction(0.3),
                                        outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST)
                                ),
                                intakeController.dropLiftIntake()
                        ),
                        outtakeController.dropOnePixelAction(),
                        //new SleepAction(0.1),
                        /*
                        outtakeController.dropOnePixelAction(),
                        new SleepAction(0.2),
                         */
                        trajDropYellowPixelToDropPurplePixel,
                        intakeController.dropPurplePixelUsingIntakeAction()
                )
        );

        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    new ParallelAction(
                            new SequentialAction(
                                    //new SleepAction(0.2),
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
                                            //new SleepAction(0.2),
                                            intakeController.intakeLiftUpAction()
                                    ),
                                    outtakeController.moveDropToReadyforTransferAction()
                            ),
                            new ParallelAction(
                                    trajDropPurplePixelToStack,
                                   // new SequentialAction(
                                            new SleepAction(0.25),
                                            intakeController.dropLiftIntake()
                                   // )
                            ),
                            intakeController.intakeAtStackTwoPixelsAction(),
                            //new SleepAction(0.1),
                            new ParallelAction(
                                    trajStackToDropStackPixel,
                                    new SequentialAction(
                                            intakeController.intakeLiftUpAction(),
                                            outtakeController.moveReadyForTransferToTransferAction(),
                                           // new SleepAction(0.1),
                                            outtakeController.moveTransferToPickupAction(),
                                            //new SleepAction(0.2),
                                            outtakeController.moveTransferToReadyForTransferAction()
                                    )
                            ),
                            /*
                            outtakeController.moveTransferToPickupAction(),
                            new SleepAction(0.2),
                            outtakeController.moveTransferToReadyForTransferAction(),
                             */
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE),
                            new SleepAction(0.1),
                            outtakeController.dropOnePixelAction(),
                            new SleepAction(0.15),
                            //new SleepAction(0.25),
                            outtakeController.dropOnePixelAction(),
                            new SleepAction(0.7)
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
                                        //new SleepAction(1),
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
                                //new SleepAction(0.2),
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

    public void runActionForRedLeftBlueRight(){
        Actions.runBlocking(
                new SequentialAction(
                        trajInitToDropPurplePixel,
                        new SleepAction(0.3),
                        intakeController.squishPurplePixelInStartOfAutoForDropAction(),//TODO: WHY HERE
                        intakeController.dropPurplePixelUsingIntakeAction(),
                        //new ParallelAction(
                        outtakeController.moveDropToReadyforTransferAction(),//TODO: WHY
                        intakeController.intakeLiftUpAction(),
                        new SleepAction(10),//TODO: WHY
                        trajDropPurplePixelToDropYellowPixel, //TODO: SHOULD PICK ONE FROM STACK
                        outtakeController.moveReadyForTransferToTransferAction(),
                        new SleepAction(0.1), //TODO: WHY
                        outtakeController.moveTransferToPickupAction(),
                        new SleepAction(0.3), //TODO: WHY
                        outtakeController.moveTransferToReadyForTransferAction(),
                        outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST),
                        new SleepAction(0.75), //TODO: WHY
                        outtakeController.dropOnePixelAction(),
                        new SleepAction(0.15), //TODO: WHY
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
                                    new SleepAction(1), //TODO: WHY
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
                                        new SleepAction(1), //TODO: WHY
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

        intakeController = new IntakeController(this.intake, this.magazine,this);
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
        } else {
            driveTrain.pose = startPose;
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
