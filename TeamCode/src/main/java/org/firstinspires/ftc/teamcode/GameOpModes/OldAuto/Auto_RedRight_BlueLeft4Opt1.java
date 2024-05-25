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
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
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
@Disabled
@Autonomous(name = "RedRight_BlueLeft 4 Opt 1", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")

public class Auto_RedRight_BlueLeft4Opt1 extends LinearOpMode {

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
        PRELOAD_AND_PARK,
        ONE_CYCLE,
        TWO_CYCLE
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

    public enum DROP_STACK_PIXEL_OPTION{
        BACK_STAGE,
        BACK_DROP
    }
    public static DROP_STACK_PIXEL_OPTION dropStackPixelOption = DROP_STACK_PIXEL_OPTION.BACK_DROP;

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
        //selectStartingPosition();
        GameField.startPosition = GameField.START_POSITION.RED_RIGHT;
        GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
        autoOption = AUTO_OPTION.ONE_CYCLE;
        pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
        parkingOption = PARKING_OPTION.RIGGING_WALL;
        dropStackPixelOption = DROP_STACK_PIXEL_OPTION.BACK_DROP;
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
            telemetry.addData("Selected Parking", parkingOption);
            telemetry.addData("Drop Stack Pixel Option", dropStackPixelOption);
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            //visionTfodFront.runTfodTensorFlow();
            visionOpenCV.runOpenCVObjectDetection();
            telemetry.addData("Vision identified SpikeMark Location", visionOpenCV.identifiedSpikeMarkLocation);
            telemetry.update();

            //Build parking trajectory based on last detected target by vision
            setTrajectoryBasedOnVision();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);

            runActionForRedRightBlueLeft();
        }
    }   // end runOpMode()

    Pose2d initPose= new Pose2d(0, 0, 0);
    Pose2d moveBeyondTrussPose = new Pose2d(6.5, 0, 0);

    Pose2d dropPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d afterPurplePixelPose = new Pose2d(0, 0, 0); //TODO: FIX BUG
    Pose2d afterPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d afterPurplePixelPoseWall = new Pose2d(0, 0, 0);//TODO: CHECK

    Pose2d stageDoorStackPose = new Pose2d(0, 0, 0);
    //Pose2d stageMidwayTrussPose = new Pose2d(0,0,0);

    Pose2d stageMidwayBackDropPoseLeft = new Pose2d(0, 0, 0);
    Pose2d stageMidwayBackDropPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d stageMidwayBackDropPoseRight = new Pose2d(0, 0, 0);

    Pose2d stageMidwayStackBackDropPose = new Pose2d(0, 0, 0);

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
    Action trajDropYellowPixelToDropPurplePixelToStack, trajDropYellowPixelToDropPurplePixelToStackLeft,
            trajDropYellowPixelToDropPurplePixelToStackMiddle, trajDropYellowPixelToDropPurplePixelToStackRight;
    Action trajStackToDropStackPixel, trajDropStackPixelToStack, trajStackToDropStackPixelLoop2, trajDropStackPixelToPark;
    Action trajStackToMidwayPose, trajMidwayPoseToDropStackPixel;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        switch (GameField.startPosition) {
            case BLUE_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);
                dropPurplePixelPoseLeft = new Pose2d(34, 21, Math.toRadians(-90)); //x22.2, 3.3, 25
                dropYellowPixelPoseLeft = new Pose2d(17.5, 35.4, Math.toRadians(-90));//x16.7, y32.5
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(18.7, 30, Math.toRadians(-84));
                stageMidwayBackDropPoseLeft = new Pose2d(59, 15.7, Math.toRadians(-84)); //x53

                dropPurplePixelPoseMiddle = new Pose2d(40, 12, Math.toRadians(-88)); //x28, y-1.6, 10.7
                dropYellowPixelPoseMiddle = new Pose2d(24, 35.4, Math.toRadians(-90));//x25, y35
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(25, 30, Math.toRadians(-84));
                stageMidwayBackDropPoseMiddle = new Pose2d(59, 9, Math.toRadians(-84)); //x53

                dropPurplePixelPoseRight = new Pose2d(32, -2, Math.toRadians(-88)); //22.7,-6.7,-60//x24.5, y-9, -36.7
                dropYellowPixelPoseRight = new Pose2d(31, 35.4, Math.toRadians(-88)); //y=33.5, x35
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(33.5, 30, Math.toRadians(-84));
                stageMidwayBackDropPoseRight = new Pose2d(59, 1.5, Math.toRadians(-87)); //x53

                wallStackPose = new Pose2d(31,-72.5, Math.toRadians(-59)); //-60
                wallMidwayStackPose = new Pose2d(6, -52, Math.toRadians(-85));
                wallMidwayBackDropPose = new Pose2d(5, 18, Math.toRadians(-85));
                stageDoorStackPose = new Pose2d(68, -66.5, Math.toRadians(-85));//x51, y-73.8, -90
                //stageMidwayTrussPose = new Pose2d(50, -29, Math.toRadians(-90));//x53
                stageMidwayStackBackDropPose = new Pose2d(50, 18, Math.toRadians(-87)); //x53 -86

                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                parkPoseWall = new Pose2d(2.5, 34, Math.toRadians(-90)); //x2, x26
                parkPoseStageDoor = new Pose2d(48, 32, Math.toRadians(-84)); //x50, y30

                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    if (dropStackPixelOption == DROP_STACK_PIXEL_OPTION.BACK_DROP) {
                        dropStackPixelPose = new Pose2d(20, 31.5, Math.toRadians(-86)); //y=32.5, x33.5
                    } else {//BACK_STAGE
                        dropStackPixelPose = parkPoseStageDoor;
                    }
                    //beforeParkAfterDropStackPixelPose = new Pose2d(31.5, 30, Math.toRadians(-90)); //y=33.5, x35.5
                } else { //WALL_RIGGING
                    if (dropStackPixelOption == DROP_STACK_PIXEL_OPTION.BACK_DROP) {
                        dropStackPixelPose = new Pose2d(25, 33.5, Math.toRadians(-90));//x20, y35.5
                    } else {//BACK_STAGE
                        dropStackPixelPose = parkPoseWall;

                    }
                    //beforeParkAfterDropStackPixelPose = new Pose2d(18.7, 30, Math.toRadians(-90));//x20, y35.5
                }

                break;

            case RED_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);
                dropPurplePixelPoseLeft = new Pose2d(28.5, 1.5, Math.toRadians(90));//x20.5, y8,51
                dropYellowPixelPoseLeft = new Pose2d(33, -36, Math.toRadians(88)); //x=31.5,y=-32
                beforeParkAfterDropYellowPixelPoseLeft = new Pose2d(33, -30, Math.toRadians(90));
                stageMidwayBackDropPoseLeft = new Pose2d(56.5, 1.5, Math.toRadians(85));//x52//x47, 0.7, 90

                dropPurplePixelPoseMiddle = new Pose2d(39, -13, Math.toRadians(90));//x27.6, y0, 13
                dropYellowPixelPoseMiddle = new Pose2d(26, -35.8, Math.toRadians(90));//x27, y-32
                beforeParkAfterDropYellowPixelPoseMiddle = new Pose2d(26, -30, Math.toRadians(90));
                stageMidwayBackDropPoseMiddle = new Pose2d(56.5, -15, Math.toRadians(85));//x52//x47, -17, 90

                dropPurplePixelPoseRight = new Pose2d(27.5, -20, Math.toRadians(90));//x26.8, y-21.6
                dropYellowPixelPoseRight = new Pose2d(20, -35.8, Math.toRadians(90));//x18, y-32
                beforeParkAfterDropYellowPixelPoseRight = new Pose2d(16.5, -30, Math.toRadians(90));
                stageMidwayBackDropPoseRight = new Pose2d(56.5, -20, Math.toRadians(85));//x52//x47, -17, 90

                afterPurplePixelPose = new Pose2d(8, 14, Math.toRadians(30));
                wallStackPose = new Pose2d(24.3, 74, Math.toRadians(59));//73
                wallMidwayStackPose = new Pose2d(6.7, 52, Math.toRadians(87));
                wallMidwayBackDropPose = new Pose2d(6, -10, Math.toRadians(87));
                stageDoorStackPose = new Pose2d(55.5, 69.7, Math.toRadians(86));//x45//x55.2, y72.2, 90
                stageMidwayStackBackDropPose = new Pose2d(56, -23, Math.toRadians(86));//x52//x47
                //stageMidwayTrussPose = new Pose2d(49, 27.5, Math.toRadians(90));

                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                parkPoseWall = new Pose2d(26, -34, Math.toRadians(90));
                parkPoseStageDoor = new Pose2d(56, -36, Math.toRadians(90));//x56,-38

                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    if (dropStackPixelOption == DROP_STACK_PIXEL_OPTION.BACK_DROP) {
                        dropStackPixelPose = new Pose2d(30, -35.2, Math.toRadians(90));//x20, -34.5
                    } else {//BACK_STAGE
                        dropStackPixelPose = parkPoseStageDoor;
                    }
                    beforeParkAfterDropStackPixelPose = new Pose2d(30, -30, Math.toRadians(90));
                } else { //WALL_RIGGING
                    if (dropStackPixelOption == DROP_STACK_PIXEL_OPTION.BACK_DROP) {
                        dropStackPixelPose = new Pose2d(30, -33.5, Math.toRadians(90));
                    } else {//BACK_STAGE
                        dropStackPixelPose = parkPoseWall;
                    }
                    beforeParkAfterDropStackPixelPose = new Pose2d(17, -30, Math.toRadians(90));
                }

                break;

        }

        if (parkingOption == PARKING_OPTION.STAGEDOOR) {
            parkPose = parkPoseStageDoor;
        } else {
            parkPose = parkPoseWall;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        trajInitToDropYellowPixelLeft = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropYellowPixelPoseLeft, 0, new TranslationalVelConstraint(25.0),
                    new ProfileAccelConstraint(-20.0, 20.0))
                .build();
        trajInitToDropYellowPixelMiddle = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropYellowPixelPoseMiddle, 0, new TranslationalVelConstraint(30.0),
                        new ProfileAccelConstraint(-25.0, 20.0))
                .build();
        trajInitToDropYellowPixelRight = drive.actionBuilder(initPose)
                .strafeTo(moveBeyondTrussPose.position)
                .splineToLinearHeading(dropYellowPixelPoseRight, 0, new TranslationalVelConstraint(24.0),
                        new ProfileAccelConstraint(-19.0, 19.0))
                .build();

        //FOR ONE_LOOP
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
                    .strafeToLinearHeading(stageMidwayBackDropPoseLeft.position, stageMidwayBackDropPoseLeft.heading,
                            new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15,15))
                    .strafeTo(stageDoorStackPose.position)
                    .build();
            trajDropPurplePixelToStackMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                    .strafeToLinearHeading(stageMidwayBackDropPoseMiddle.position, stageMidwayBackDropPoseMiddle.heading,
                            new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15,15))
                    .strafeTo(stageDoorStackPose.position)
                    .build();
            trajDropPurplePixelToStackRight = drive.actionBuilder(dropPurplePixelPoseRight)
                    .strafeToLinearHeading(stageMidwayBackDropPoseRight.position, stageMidwayBackDropPoseRight.heading,
                            new TranslationalVelConstraint(20), new ProfileAccelConstraint(-15,15))
                    .strafeTo(stageDoorStackPose.position)
                    .build();

            trajStackToDropStackPixel = drive.actionBuilder(stageDoorStackPose)
                    .setReversed(true)
                    .strafeToLinearHeading(stageMidwayStackBackDropPose.position,stageMidwayStackBackDropPose.heading/*,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-30,30)*/)
                    .splineToLinearHeading(dropStackPixelPose, 0, new TranslationalVelConstraint(40),
                            new ProfileAccelConstraint(-30,30))
                            //new TranslationalVelConstraint(40), new ProfileAccelConstraint(-10,10))
                    .build();

            trajStackToDropStackPixelLoop2 = drive.actionBuilder(stageDoorStackPose)
                    .strafeTo(stageMidwayStackBackDropPose.position)
                    .setReversed(true)
                    .splineToLinearHeading(dropStackPixelPose, 0)
                    .build();

            trajDropStackPixelToStack = drive.actionBuilder(dropStackPixelPose)
                    .splineToLinearHeading(stageMidwayStackBackDropPose, 0)
                    .strafeTo(stageDoorStackPose.position)
                    .build();

            trajDropStackPixelToPark = drive.actionBuilder(dropStackPixelPose)
                    //.strafeToLinearHeading(beforeParkAfterDropStackPixelPose.position,beforeParkAfterDropStackPixelPose.heading)
                    .strafeToLinearHeading(parkPoseStageDoor.position,parkPoseStageDoor.heading,new TranslationalVelConstraint(20.0))
                    //.splineToLinearHeading(parkPoseStageDoor, 0)
                    .build();
        } else {
            trajDropPurplePixelToStackLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position, wallMidwayBackDropPose.heading)
                    .strafeTo(wallMidwayStackPose.position)
                    .strafeToLinearHeading(wallStackPose.position, wallStackPose.heading)
                    .build();
            trajDropPurplePixelToStackMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position, wallMidwayBackDropPose.heading)
                    .strafeTo(wallMidwayStackPose.position)
                    .strafeToLinearHeading(wallStackPose.position, wallStackPose.heading)
                    .build();
            trajDropPurplePixelToStackRight = drive.actionBuilder(dropPurplePixelPoseRight)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position, wallMidwayBackDropPose.heading)
                    .strafeTo(wallMidwayStackPose.position)
                    .strafeToLinearHeading(wallStackPose.position, wallStackPose.heading)
                    .build();


            trajStackToDropStackPixel = drive.actionBuilder(wallStackPose)
                    .setReversed(true)
                    .strafeToLinearHeading(wallMidwayStackPose.position,wallMidwayStackPose.heading,
                            new TranslationalVelConstraint(20), new ProfileAccelConstraint(-27,27))
                    .strafeToLinearHeading(wallMidwayBackDropPose.position, wallMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(32), new ProfileAccelConstraint(-27,27))
                    .splineToLinearHeading(dropStackPixelPose, 0)
                    .build();

            trajStackToMidwayPose = drive.actionBuilder(wallStackPose)
                    .setReversed(true)
                    .strafeToLinearHeading(wallMidwayStackPose.position,wallMidwayStackPose.heading,
                            new TranslationalVelConstraint(35), new ProfileAccelConstraint(-30,30))
                    .strafeToLinearHeading(wallMidwayBackDropPose.position, wallMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(35), new ProfileAccelConstraint(-30,30))
                    .build();

            trajMidwayPoseToDropStackPixel = drive.actionBuilder(wallMidwayBackDropPose)
                    .setReversed(true)
                    .splineToLinearHeading(dropStackPixelPose, 0)
                    .build();


            trajDropStackPixelToPark = drive.actionBuilder(dropStackPixelPose)
                    //.strafeTo(beforeParkAfterDropStackPixelPose.position)
                    .strafeToLinearHeading(parkPoseWall.position, parkPoseWall.heading,new TranslationalVelConstraint(20.0))
                    //.splineToLinearHeading(parkPoseStageDoor, 0)
                    .build();

        }

    }

    public void setTrajectoryBasedOnVision() {
        switch (visionOpenCV.identifiedSpikeMarkLocation) {
            case LEFT:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelLeft;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelLeft;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkLeft;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackLeft;
                trajDropYellowPixelToDropPurplePixelToStack = trajDropYellowPixelToDropPurplePixelToStackLeft;
                break;
            case MIDDLE:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelMiddle;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelMiddle;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkMiddle;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackMiddle;
                trajDropYellowPixelToDropPurplePixelToStack = trajDropYellowPixelToDropPurplePixelToStackMiddle;
                break;
            case RIGHT:
                trajInitToDropYellowPixel = trajInitToDropYellowPixelRight;
                trajDropYellowPixelToDropPurplePixel = trajDropYellowPixelToDropPurplePixelRight;
                trajDropPurplePixelToPark = trajDropPurplePixelToParkRight;
                trajDropPurplePixelToStack = trajDropPurplePixelToStackRight;
                trajDropYellowPixelToDropPurplePixelToStack = trajDropYellowPixelToDropPurplePixelToStackRight;
                break;
        }
    }

    public void runActionForRedRightBlueLeft() {
        if (autoOption == AUTO_OPTION.PRELOAD_AND_PARK) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                    trajInitToDropYellowPixel,
                                    new SequentialAction(
                                            new SleepAction(0.9),
                                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST_AUTO)
                                    )
                            ),
                            new SleepAction(0.5),
                            new ParallelAction(
                                    outtakeController.dropOnePixelAction(),
                                    intakeController.dropLiftIntake()
                            ),
                            trajDropYellowPixelToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.5), //ADD FOR SYNCHRONIZING TIME
                            //Go to Park
                            new ParallelAction(
                                    intakeController.intakeLiftUpAction(),
                                    trajDropPurplePixelToPark,
                                    outtakeController.moveOuttakeToEndStateAction()
                            )
                    )
            );
        }

        if (autoOption == AUTO_OPTION.ONE_CYCLE) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                    trajInitToDropYellowPixel,
                                    new SequentialAction(
                                            new SleepAction(0.9),
                                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST_AUTO)
                                    )
                            ),
                            new SleepAction(0.7),
                            new ParallelAction(
                                    outtakeController.dropOnePixelAction(),
                                    intakeController.dropLiftIntake()
                            ),
                            trajDropYellowPixelToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.5),
                            //LOOP 1
                            new ParallelAction(
                                    intakeController.intakeLiftUpAction(),
                                    outtakeController.moveDropToReadyforTransferAction()
                            ),
                            new ParallelAction(
                                    trajDropPurplePixelToStack,
                                    new SequentialAction(
                                            new SleepAction(1.5),
                                            intakeController.dropLiftIntake()
                                    )
                            ),
                            //intakeController.intakeAtStackTwoPixelsAction(),
                            intakeController.intakeAtStackUsingMagazineSensorAction(1.35),//1.5
                            new ParallelAction(
                                    intakeController.reverseStackIntakeOneRotationAction(),
                                    intakeController.intakeReverseAction(1.4)
                            ),
                            intakeController.intakeLiftUpAction(),
                            trajStackToMidwayPose,
                            new SequentialAction(
                                    new SleepAction(0.5),
                                    outtakeController.moveReadyForTransferToTransferAction(),
                                    new SleepAction(0.2),
                                    outtakeController.moveTransferToPickupAction(),
                                    new SleepAction(0.2),
                                    outtakeController.movePickupToReadyForTransferAction()
                            ),
                            trajMidwayPoseToDropStackPixel,
                            //trajStackToDropStackPixel,
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_WHITE_AUTO),
                            new SleepAction(0.7),
                            /*outtakeController.dropOnePixelAction(),
                            new SleepAction(0.4),
                            outtakeController.dropOnePixelAction(),
                            new SleepAction(0.4),*/
                            outtakeController.dropTwoPixelAction(),
                            new SleepAction(0.2),
                            //Go to Park
                            new ParallelAction(
                                    trajDropStackPixelToPark,
                                    outtakeController.moveOuttakeToEndStateAction()
                            )
                    )
            );
        }

        if (autoOption == AUTO_OPTION.TWO_CYCLE) {
            Actions.runBlocking(
                    new SequentialAction(
                            new ParallelAction(
                                    intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                                    trajInitToDropYellowPixel,
                                    new SequentialAction(
                                            new SleepAction(0.5),
                                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST_AUTO)
                                    ),
                                    intakeController.dropLiftIntake()
                            ),
                            outtakeController.dropOnePixelAction(),
                            //NEW LOOP 1
                            trajDropYellowPixelToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.7),
                            new ParallelAction(
                                    intakeController.intakeLiftUpAction(),
                                    outtakeController.moveDropToReadyforTransferAction(),
                                    trajDropPurplePixelToStack,
                                    new SequentialAction(
                                            new SleepAction(2),
                                            intakeController.dropLiftIntake()
                                    )
                            ),
                            intakeController.intakeAtStackTwoPixelsAction(),
                            new ParallelAction(
                                    trajStackToDropStackPixel,
                                    intakeController.intakeLiftUpAction(),
                                    new SequentialAction(
                                            new SleepAction(3),
                                            outtakeController.moveReadyForTransferToTransferAction(),
                                            outtakeController.moveTransferToPickupAction(),
                                            outtakeController.movePickupToReadyForTransferAction(),
                                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST)
                                    )
                            ),
                            new SequentialAction(
                                    outtakeController.dropOnePixelAction(),
                                    new SleepAction(0.2),
                                    outtakeController.dropOnePixelAction(),
                                    new SleepAction(0.1)
                            ),

                            //LOOP 2
                            new ParallelAction(
                                    trajDropStackPixelToStack,
                                    outtakeController.moveDropToReadyforTransferAction(),
                                    new SequentialAction(
                                            new SleepAction(2),
                                            intakeController.dropLiftIntake()
                                    )
                            ),
                            intakeController.intakeAtStackTwoPixelsAction(),
                            new ParallelAction(
                                    trajStackToDropStackPixelLoop2,
                                    intakeController.intakeLiftUpAction(),
                                    new SequentialAction(
                                            new SleepAction(3),
                                            outtakeController.moveReadyForTransferToTransferAction(),
                                            outtakeController.moveTransferToPickupAction(),
                                            outtakeController.movePickupToReadyForTransferAction(),
                                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOW_LINE)
                                    )
                            ),
                            new SequentialAction(
                                    outtakeController.dropOnePixelAction(),
                                    new SleepAction(0.2),
                                    outtakeController.dropOnePixelAction(),
                                    new SleepAction(0.1)
                            ),
                            //Go to Park
                            new ParallelAction(
                                    trajDropStackPixelToPark,
                                    outtakeController.moveOuttakeToEndStateAction()
                            )
                    )
            );
        }
    }

    /*public Action dropPurplePixelUsingIntakeWithWaitActionRRBL(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                switch (visionOpenCV.identifiedSpikeMarkLocation) {
                    case LEFT:
                        intakeController.dropPurplePixelUsingIntakeWithWait(1.5);
                        break;
                    case MIDDLE:
                        intakeController.dropPurplePixelUsingIntakeWithWait(1);
                        break;
                    case RIGHT:
                        intakeController.dropPurplePixelUsingIntakeWithWait(1);
                        break;
                }

                return false;
            }
        };
    }*/

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        //telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:","");
            telemetry.addData("    Red Right  ", "(A / X)");
            telemetry.addData("    Blue Left   ", "(X / ▢)");
            if(gamepadController.gp1GetCrossPress()){
                GameField.startPosition = GameField.START_POSITION.RED_RIGHT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if(gamepadController.gp1GetSquarePress()){
                GameField.startPosition = GameField.START_POSITION.BLUE_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                break;
            }

            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addLine("Select Auto Options");
            telemetry.addData("    Yellow, Purple and Park     ", "(X / ▢)");
            telemetry.addData("    1 cycle                     ", "(Y / Δ)");
            //telemetry.addData("    2 cycle (only Stage Door)   ", "(B / O)");

            if (gamepadController.gp1GetSquarePress()) {
                autoOption = AUTO_OPTION.PRELOAD_AND_PARK;
                break;
            }
            if (gamepadController.gp1GetTrianglePress()) {
                autoOption = AUTO_OPTION.ONE_CYCLE;
                break;
            }
            /*if (gamepadController.gp1GetCirclePress()) {
                autoOption = AUTO_OPTION.TWO_CYCLE;
                pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                parkingOption = PARKING_OPTION.STAGEDOOR;
                break;
            }*/
            telemetry.update();
        }

        if (autoOption != AUTO_OPTION.TWO_CYCLE) {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addLine("Select Pathway and Parking option");
                telemetry.addData("    Wall Rigging ", "Y / Δ");
                telemetry.addData("    Stage Door   ", "B / O ");
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

        if (autoOption != AUTO_OPTION.PRELOAD_AND_PARK) {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addData("Selected Pathway and Parking option", pathwayOption);
                telemetry.addLine("Drop Stack pixels on");
                telemetry.addData("    Back Drop ", "Y / Δ");
                telemetry.addData("    Back Stage", "B / O ");
                if (gamepadController.gp1GetButtonYPress()) {
                    dropStackPixelOption = DROP_STACK_PIXEL_OPTION.BACK_DROP;
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    dropStackPixelOption = DROP_STACK_PIXEL_OPTION.BACK_STAGE;
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
        //visionTfodFront = new VisionTfod(hardwareMap, telemetry, "Webcam 1");
        //telemetry.addLine("VisionTfod Initialized");
        visionOpenCV = new VisionOpenCV(hardwareMap, telemetry, "Webcam 1");
        telemetry.addLine("VisionOpenCV Initialized");
        telemetry.update();

        /* Create Lights */
        lights = new Lights(hardwareMap, telemetry);
        telemetry.addLine("Lights Initialized");
        lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);
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
        GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;

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
