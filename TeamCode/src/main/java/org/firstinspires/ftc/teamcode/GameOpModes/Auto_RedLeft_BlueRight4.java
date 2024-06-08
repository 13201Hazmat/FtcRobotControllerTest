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
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.Controllers.IntakeController;
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
import org.firstinspires.ftc.teamcode.SubSystems.ParkingArm;
import org.firstinspires.ftc.teamcode.SubSystems.VisionOpenCV;
import org.firstinspires.ftc.teamcode.SubSystems.VisionSensor;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "RedLeft_BlueRight 4", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
public class Auto_RedLeft_BlueRight4 extends LinearOpMode {

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Magazine magazine;
    public OuttakeSlides outtakeSlides;
    public OuttakeArm outtakeArm;
    public Climber climber;
    public Launcher launcher;
    public ParkingArm parkingArm;
    public VisionSensor visionSensor;
    public VisionOpenCV visionOpenCV;
    public Lights lights;
    public OuttakeController outtakeController;
    public IntakeController intakeController;
    public TelemetryPacket telemetryPacket = new TelemetryPacket();

    public MecanumDrive drive;

    //Static Class for knowing system state

    public enum AUTO_OPTION{
        PURPLE_AND_PARK,
        PURPLE_STACK_AND_PARK,
        PRELOAD_STACK1_AND_PARK,
    }
    public static AUTO_OPTION autoOption;

    public enum PATHWAY_OPTION{
        RIGGING_WALL,
        STAGEDOOR
    }
    public static PATHWAY_OPTION pathwayOption;

    public enum PARKING_OPTION{
        WALL_RIGGING,
        WALL_RIGGING_FRONT,
        STAGEDOOR,
        STAGEDOOR_FRONT
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
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEFAULT);

            outtakeArm.openGrip();
            runActionForRedLeftBlueRight();

        }
    }   // end runOpMode()

    Pose2d initPose= new Pose2d(0, 0, 0);

    Pose2d dropPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d dropPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d dropPurplePixelPoseWallLeft = new Pose2d(0,0,0);
    Pose2d dropPurplePixelPoseWallMiddle = new Pose2d(0,0,0);
    Pose2d dropPurplePixelPoseWallRight = new Pose2d(0,0,0);

    Pose2d afterPurplePixelPoseLeft = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseMiddle = new Pose2d(0, 0, 0);
    Pose2d afterPurplePixelPoseRight = new Pose2d(0, 0, 0);

    Pose2d afterPurplePixelPoseWall = new Pose2d(0, 0, 0);//TODO: CHECK
    Pose2d afterStackPoseStageDoor = new Pose2d(0, 0, 0);//TODO: CHECK
    Pose2d afterStackPoseWall = new Pose2d(0, 0, 0);

    Pose2d stageDoorStackPose = new Pose2d(0, 0, 0);
    Pose2d stageMidwayBackDropPose = new Pose2d(0, 0, 0);
    Pose2d wallStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayStackPose = new Pose2d(0, 0, 0);
    Pose2d wallMidwayBackDropPose = new Pose2d(0, 0, 0);

    Pose2d dropYellowPixelPoseLeft= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseMiddle= new Pose2d(0, 0, 0);
    Pose2d dropYellowPixelPoseRight= new Pose2d(0, 0, 0);

    Pose2d beforeParkAfterDropYellowPixelPose = new Pose2d(0,0,0);
    Pose2d beforeParkMiddleAfterDropYellowPixelPose = new Pose2d(0,0,0);
    Pose2d beforeParkWallAfterDropYellowPixelPose = new Pose2d(0,0,0);

    Pose2d dropStackPixelPose = new Pose2d(0, 0, 0);
    Pose2d dropStackPixelPoseBackDrop = new Pose2d(0, 0, 0);

    Pose2d parkPoseStageDoor = new Pose2d(0, 0, 0);
    Pose2d parkPoseStageDoorFront = new Pose2d(0, 0, 0);
    Pose2d parkPoseWall = new Pose2d(0, 0, 0);
    Pose2d parkPoseWallFront = new Pose2d(0, 0, 0);
    Pose2d parkPose = new Pose2d(0, 0, 0);


    //AUTO_OPTION.PURPLE_AND_PARK
    Action trajInitToDropPurplePixel, trajInitToDropPurplePixelLeft, trajInitToDropPurplePixelMiddle, trajInitToDropPurplePixelRight;
    Action trajDropPurplePixelToPark, trajDropPurplePixelToParkLeft, trajDropPurplePixelToParkMiddle, trajDropPurplePixelToParkRight;

    //AUTO_OPTION.PURPLE_YELLOW_AND_PARK
    Action trajDropPurplePixelToAfterPurplePixel, trajDropPurplePixelToAfterPurplePixelLeft,
            trajDropPurplePixelToAfterPurplePixelMiddle, trajDropPurplePixelToAfterPurplePixelRight;
    Action trajAfterStackToDropYellowPixel, trajAfterStackToDropYellowPixelLeft,
            trajAfterStackToDropYellowPixelMiddle, trajAfterStackToDropYellowPixelRight;
    Action trajDropYellowPixelToPark, trajDropYellowPixelLeftToPark, trajDropYellowPixelMiddleToPark, trajDropYellowPixelRightToPark;
    Action trajAfterStackToPark;


    //AUTO_OPTION.PRELOAD_STACK1_AND_PARK
    Action trajAfterPurplePixelToStack, trajAfterPurplePixelToStackLeft, trajAfterPurplePixelToStackMiddle,trajAfterPurplePixelToStackRight;
    Action trajStackToAfterStack;

    public void buildAutonoumousMode() {
        //Initialize Pose2d as desired
        double waitSecondsBeforeDrop = 0;

        switch (GameField.startPosition) {
            case BLUE_RIGHT:
                drive = new MecanumDrive(hardwareMap, initPose);

                dropPurplePixelPoseLeft = new Pose2d(22.2, 7.5, Math.toRadians(47.8));//23.5, 9, 62
                dropPurplePixelPoseWallLeft = new Pose2d(20, 3.5, Math.toRadians(60));
                dropYellowPixelPoseLeft = new Pose2d(16, 136, Math.toRadians(-90));//x18, y87
                afterPurplePixelPoseLeft = new Pose2d(12, -4, Math.toRadians(34)); //x17, y-4,-90

                dropPurplePixelPoseMiddle = new Pose2d(27, 5.4, Math.toRadians(19)); //x26.4, y7.8, 2
                dropPurplePixelPoseWallMiddle = new Pose2d(26, 0, Math.toRadians(0));
                dropYellowPixelPoseMiddle = new Pose2d(20, 136, Math.toRadians(-90)); //x24.6, y88
                afterPurplePixelPoseMiddle = new Pose2d(7.3, -4, Math.toRadians(18));//17, -4, -9

                dropPurplePixelPoseRight = new Pose2d(42.3, 1.4, Math.toRadians(-131.3)); //x25.5, y-2.5, -52.5
                dropPurplePixelPoseWallRight = new Pose2d(13.5, -6.6, Math.toRadians(0));//22.5, -1.8, -46
                dropYellowPixelPoseRight = new Pose2d(24, 136, Math.toRadians(-90)); //x30.5, 88
                afterPurplePixelPoseRight = new Pose2d(48, 3.4, Math.toRadians(-88)); //x15, y5, -90degrees

                afterPurplePixelPoseWall = new Pose2d(2, 0, Math.toRadians(-92));//x18,y4,-30
                afterStackPoseStageDoor = new Pose2d(48, 50, Math.toRadians(-90));
                afterStackPoseWall = new Pose2d(2, -3.1, Math.toRadians(-90));//x18,y4,-30
                wallStackPose = new Pose2d(21, -19, Math.toRadians(-60)); //x19.8,19,60
                wallMidwayStackPose = new Pose2d(2, 0, Math.toRadians(-90));//x2, y-5
                wallMidwayBackDropPose = new Pose2d(2, 116, Math.toRadians(-90));//x2, y-73
                stageDoorStackPose = new Pose2d(51, -19, Math.toRadians(-90));//x49.2, 19.2
                //stageMidwayTrussPose = new Pose2d(50.4, -20.6, 90);//x48.5, y27.5, 90
                stageMidwayBackDropPose = new Pose2d(46, 122, Math.toRadians(-90));//x49
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                beforeParkWallAfterDropYellowPixelPose = new Pose2d(2,115,-90);
                parkPoseWall = new Pose2d(2, 141, Math.toRadians(-90)); //x-1, y-81
                parkPoseWallFront = new Pose2d(2, 120, Math.toRadians(-90));
                beforeParkMiddleAfterDropYellowPixelPose = new Pose2d(42,115,-90);
                parkPoseStageDoor = new Pose2d(46, 131, Math.toRadians(-90));//x46, y-79
                parkPoseStageDoorFront = new Pose2d(46, 120, Math.toRadians(-45));

                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    dropStackPixelPoseBackDrop = new Pose2d(26, 131, Math.toRadians(-90)); //y=33.5, x35.5
                } else { //WALL_RIGGING
                    dropStackPixelPoseBackDrop = new Pose2d(31.5, 131, Math.toRadians(-90));//x20, y35.5
                }

                break;

            case RED_LEFT:
                drive = new MecanumDrive(hardwareMap, initPose);

                dropPurplePixelPoseLeft = new Pose2d(40, -3.5, Math.toRadians(110)); //x20.8, y3.3, 31.5
                dropPurplePixelPoseWallLeft = new Pose2d(14.5, 9, Math.toRadians(0));
                dropYellowPixelPoseLeft = new Pose2d(36, -137, Math.toRadians(90));//x32, y-91
                afterPurplePixelPoseLeft = new Pose2d(49, -4, Math.toRadians(90));

                dropPurplePixelPoseMiddle = new Pose2d(25.8, -5, Math.toRadians(-13.8)); //-2.3
                dropPurplePixelPoseWallMiddle = new Pose2d(24.2, 3, Math.toRadians(0)); //3
                dropYellowPixelPoseMiddle = new Pose2d(32, -136, Math.toRadians(90)); //x28,y=-91
                afterPurplePixelPoseMiddle = new Pose2d(18, 4, Math.toRadians(-30));//x18,y4,-30

                dropPurplePixelPoseRight = new Pose2d(24, -4.4, Math.toRadians(-60)); //x25.1, y-5.3, -71
                dropPurplePixelPoseWallRight = new Pose2d(24, -4.4, Math.toRadians(-60));
                dropYellowPixelPoseRight = new Pose2d(26, -137, Math.toRadians(90));//x23 y=-88
                afterPurplePixelPoseRight = new Pose2d(18, 4, Math.toRadians(-30));//x18,y4,-30

                afterPurplePixelPoseWall = new Pose2d(4, 3.1, Math.toRadians(92));//x18,y4,-30
                afterStackPoseStageDoor = new Pose2d(52, -50, Math.toRadians(90));
                afterStackPoseWall = new Pose2d(4, 3.1, Math.toRadians(92));//x18,y4,-30
                wallStackPose = new Pose2d(21, 19, Math.toRadians(60)); //x19.8,19,60
                wallMidwayStackPose = new Pose2d(4, 0, Math.toRadians(90));//x2, y-5
                wallMidwayBackDropPose = new Pose2d(4, -110, Math.toRadians(90));//x2, y-73
                stageDoorStackPose = new Pose2d(50.1, 18.5, Math.toRadians(90));//x49.2, 19.2
                //stageMidwayTrussPose = new Pose2d(50.4, -20.6, 90);//x48.5, y27.5, 90
                stageMidwayBackDropPose = new Pose2d(50.4, -122, Math.toRadians(90));//x49
                waitSecondsBeforeDrop = 2; //TODO: Adjust time to wait for alliance partner to move from board

                beforeParkMiddleAfterDropYellowPixelPose = new Pose2d(4, -115, Math.toRadians(90));
                parkPoseWall = new Pose2d(4, -141, Math.toRadians(90)); //x-1, y-81
                parkPoseWallFront = new Pose2d(4, -120, Math.toRadians(90));
                beforeParkWallAfterDropYellowPixelPose = new Pose2d(42, -115, Math.toRadians(90));
                parkPoseStageDoor = new Pose2d(42, -131, Math.toRadians(90));//x46, y-79
                parkPoseStageDoorFront = new Pose2d(42, -120, Math.toRadians(90));

                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    dropStackPixelPoseBackDrop = new Pose2d(26, -131, Math.toRadians(90));//x16,-89
                } else { //WALL_RIGGING
                    dropStackPixelPoseBackDrop = new Pose2d(36.1, -131, Math.toRadians(90));
                }

                break;
        }

        if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
            beforeParkAfterDropYellowPixelPose = beforeParkMiddleAfterDropYellowPixelPose;
            if (parkingOption == PARKING_OPTION.STAGEDOOR) {
                parkPose = parkPoseStageDoor;
            } else if (parkingOption == PARKING_OPTION.STAGEDOOR_FRONT) {
                parkPose = parkPoseStageDoorFront;
                parkingArm.deploy = true;
            }
        } else { //WALL_RIGGING
            beforeParkAfterDropYellowPixelPose = beforeParkWallAfterDropYellowPixelPose;
            if (parkingOption == PARKING_OPTION.WALL_RIGGING) {
                parkPose = parkPoseWall;
            } else if (parkingOption == PARKING_OPTION.WALL_RIGGING_FRONT) {
                parkPose = parkPoseWallFront;
                parkingArm.deploy = true;
            }
        }

        if (dropStackPixelOption == DROP_STACK_PIXEL_OPTION.BACK_DROP) {
            dropStackPixelPose = dropStackPixelPoseBackDrop;
        } else {//BACK_STAGE
            dropStackPixelPose = parkPose;
            beforeParkAfterDropYellowPixelPose = adjustPos(parkPose,-1,0);
            dropYellowPixelPoseLeft = adjustPos(parkPose,0.8,0);;
            dropYellowPixelPoseMiddle = adjustPos(parkPose,0.8,0);;
            dropYellowPixelPoseRight = adjustPos(parkPose,0.8,0);;
        }

        telemetry.addLine("+++++ After Pose Assignments ++++++");
        telemetry.update();

        //For RED_LEFT & BLUE_RIGHT
        trajDropYellowPixelLeftToPark = drive.actionBuilder(dropYellowPixelPoseLeft)
                .strafeToLinearHeading(beforeParkAfterDropYellowPixelPose.position, beforeParkAfterDropYellowPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading,
                        new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
                .build();
        trajDropYellowPixelMiddleToPark = drive.actionBuilder(dropYellowPixelPoseMiddle)
                .strafeToLinearHeading(beforeParkAfterDropYellowPixelPose.position, beforeParkAfterDropYellowPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading,
                        new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
                .build();
        trajDropYellowPixelRightToPark = drive.actionBuilder(dropYellowPixelPoseRight)
                .strafeToLinearHeading(beforeParkAfterDropYellowPixelPose.position, beforeParkAfterDropYellowPixelPose.heading)
                .strafeToLinearHeading(parkPose.position, parkPose.heading,
                        new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
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

            //FOR BLUE_RIGHT & RED_LEFT
            trajDropPurplePixelToAfterPurplePixelLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                    .strafeToLinearHeading(afterPurplePixelPoseLeft.position, afterPurplePixelPoseLeft.heading)
                    .build();
            trajDropPurplePixelToAfterPurplePixelMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                    .strafeToLinearHeading(afterPurplePixelPoseMiddle.position, afterPurplePixelPoseMiddle.heading)
                    .build();
            trajDropPurplePixelToAfterPurplePixelRight = drive.actionBuilder(dropPurplePixelPoseRight)
                    .strafeToLinearHeading(afterPurplePixelPoseRight.position, afterPurplePixelPoseRight.heading)
                    .build();

            trajAfterPurplePixelToStackLeft = drive.actionBuilder(afterPurplePixelPoseLeft)
                    .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                    .build();
            trajAfterPurplePixelToStackMiddle = drive.actionBuilder(afterPurplePixelPoseMiddle)
                    .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                    .build();
            trajAfterPurplePixelToStackRight = drive.actionBuilder(afterPurplePixelPoseRight)
                    .strafeToLinearHeading(stageDoorStackPose.position, stageDoorStackPose.heading)
                    .build();


            trajStackToAfterStack =  drive.actionBuilder(stageDoorStackPose)
                    .strafeToLinearHeading(afterStackPoseStageDoor.position, afterStackPoseStageDoor.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .build();

            trajAfterStackToDropYellowPixelLeft = drive.actionBuilder(afterStackPoseStageDoor)
                    .setReversed(true)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .strafeToLinearHeading(dropYellowPixelPoseLeft.position,dropYellowPixelPoseLeft.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .build();
            trajAfterStackToDropYellowPixelMiddle = drive.actionBuilder(afterStackPoseStageDoor)
                    .setReversed(true)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .strafeToLinearHeading(dropYellowPixelPoseMiddle.position,dropYellowPixelPoseMiddle.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .build();
            trajAfterStackToDropYellowPixelRight = drive.actionBuilder(afterStackPoseStageDoor)
                    .setReversed(true)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .strafeToLinearHeading(dropYellowPixelPoseRight.position, dropYellowPixelPoseRight.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .build();

            trajAfterStackToPark = drive.actionBuilder(afterStackPoseStageDoor)
                    .setReversed(true)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .strafeToLinearHeading(parkPose.position,parkPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .build();

            trajDropPurplePixelToParkLeft = drive.actionBuilder(dropPurplePixelPoseLeft)
                    .splineToLinearHeading(afterPurplePixelPoseLeft, 0)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .strafeTo(stageMidwayBackDropPose.position)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkMiddle = drive.actionBuilder(dropPurplePixelPoseMiddle)
                    .splineToLinearHeading(afterPurplePixelPoseMiddle, 0)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkRight = drive.actionBuilder(dropPurplePixelPoseRight)
                    .splineToLinearHeading(afterPurplePixelPoseRight, 0)
                    .strafeToLinearHeading(stageMidwayBackDropPose.position,stageMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(25), new ProfileAccelConstraint(-18,18))
                    .splineToLinearHeading(parkPose, 0)
                    .build();

        } else {
            //WALL_RIGGING
            trajInitToDropPurplePixelLeft = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(dropPurplePixelPoseWallLeft, 0)
                    .build();
            trajInitToDropPurplePixelMiddle = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(dropPurplePixelPoseWallMiddle, 0)
                    .build();
            trajInitToDropPurplePixelRight = drive.actionBuilder(drive.pose)
                    .splineToLinearHeading(dropPurplePixelPoseWallRight, 0)
                    .build();

            trajDropPurplePixelToAfterPurplePixelLeft = drive.actionBuilder(dropPurplePixelPoseWallLeft)
                    .setReversed(true)
                    .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
                    .build();
            trajDropPurplePixelToAfterPurplePixelMiddle = drive.actionBuilder(dropPurplePixelPoseWallMiddle)
                    .setReversed(true)
                    .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
                    .build();
            trajDropPurplePixelToAfterPurplePixelRight = drive.actionBuilder(dropPurplePixelPoseWallRight)
                    .setReversed(true)
                    .strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-25,25))
                    .build();

            trajAfterPurplePixelToStack = drive.actionBuilder(afterPurplePixelPoseWall)
                    .strafeToLinearHeading(wallStackPose.position, wallStackPose.heading,
                    //.splineToLinearHeading(wallStackPose,0,
                            new TranslationalVelConstraint(30), new ProfileAccelConstraint(-15,15))
                    .build();

            trajStackToAfterStack = drive.actionBuilder(wallStackPose)
                    .setReversed(true)
                    //.strafeToLinearHeading(afterPurplePixelPoseWall.position, afterPurplePixelPoseWall.heading,
                    .splineToLinearHeading(afterPurplePixelPoseWall, 0,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-15,15))
                    .build();

            trajAfterStackToDropYellowPixelLeft = drive.actionBuilder(afterPurplePixelPoseWall)
                    .setReversed(true)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position,wallMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,20))
                    .splineToLinearHeading(dropYellowPixelPoseLeft, 0)
                    .build();
            trajAfterStackToDropYellowPixelMiddle = drive.actionBuilder(afterPurplePixelPoseWall)
                    .setReversed(true)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position,wallMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,20))
                    .splineToLinearHeading(dropYellowPixelPoseMiddle, 0)
                    .build();
            trajAfterStackToDropYellowPixelRight = drive.actionBuilder(afterPurplePixelPoseWall)
                    .setReversed(true)
                    .strafeToLinearHeading(wallMidwayBackDropPose.position,wallMidwayBackDropPose.heading,
                            new TranslationalVelConstraint(40), new ProfileAccelConstraint(-20,20))
                    .splineToLinearHeading(dropYellowPixelPoseRight, 0)
                    .build();

            trajDropPurplePixelToParkLeft = drive.actionBuilder(dropPurplePixelPoseWallLeft)
                    .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                    .strafeTo(wallMidwayBackDropPose.position)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkMiddle = drive.actionBuilder(dropPurplePixelPoseWallMiddle)
                    .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                    .strafeTo(wallMidwayBackDropPose.position)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
            trajDropPurplePixelToParkRight = drive.actionBuilder(dropPurplePixelPoseWallRight)
                    .splineToLinearHeading(afterPurplePixelPoseWall, 0)
                    .strafeTo(wallMidwayBackDropPose.position)
                    .splineToLinearHeading(parkPose, 0)
                    .build();
        }

    }

    public void setTrajectoryBasedOnVision() {
        switch (visionOpenCV.identifiedSpikeMarkLocation) {
            case LEFT:
                trajDropPurplePixelToPark = trajDropPurplePixelToParkLeft;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelLeft;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelLeft;
                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    trajAfterPurplePixelToStack = trajAfterPurplePixelToStackLeft;
                }
                trajAfterStackToDropYellowPixel = trajAfterStackToDropYellowPixelLeft;
                trajDropYellowPixelToPark = trajDropYellowPixelLeftToPark;
                break;
            case MIDDLE:
                trajDropPurplePixelToPark = trajDropPurplePixelToParkMiddle;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelMiddle;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelMiddle;
                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    trajAfterPurplePixelToStack = trajAfterPurplePixelToStackMiddle;
                }
                trajAfterStackToDropYellowPixel = trajAfterStackToDropYellowPixelMiddle;
                trajDropYellowPixelToPark = trajDropYellowPixelMiddleToPark;
                break;
            case RIGHT:
                trajDropPurplePixelToPark = trajDropPurplePixelToParkRight;
                trajInitToDropPurplePixel = trajInitToDropPurplePixelRight;
                trajDropPurplePixelToAfterPurplePixel = trajDropPurplePixelToAfterPurplePixelRight;
                if (pathwayOption == PATHWAY_OPTION.STAGEDOOR) {
                    trajAfterPurplePixelToStack = trajAfterPurplePixelToStackRight;
                }
                trajAfterStackToDropYellowPixel = trajAfterStackToDropYellowPixelRight;
                trajDropYellowPixelToPark = trajDropYellowPixelRightToPark;
                break;
        }
    }

    public void runActionForRedLeftBlueRight() {
        if (autoOption == AUTO_OPTION.PURPLE_AND_PARK) {
            Actions.runBlocking(
                    new SequentialAction(
                            intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                            intakeController.dropLiftIntake(),
                            new SleepAction(0.5),
                            trajInitToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.5),
                            intakeController.intakeLiftUpAction(),
                            new SleepAction(0.5),
                            trajDropPurplePixelToPark,
                            outtakeController.moveOuttakeToEndStateAction()
                    )
            );
        }

        if (autoOption == AUTO_OPTION.PRELOAD_STACK1_AND_PARK) {
            Actions.runBlocking(
                    new SequentialAction(
                            intakeController.squishPurplePixelInStartOfAutoForDropAction(),
                            intakeController.dropLiftIntake(),
                            trajInitToDropPurplePixel,
                            intakeController.dropPurplePixelUsingIntakeAction(),
                            new SleepAction(0.3),
                            intakeController.intakeLiftUpAction(),
                            new SleepAction(0.3),
                            trajDropPurplePixelToAfterPurplePixel,
                            intakeController.dropLiftIntake(),
                            trajAfterPurplePixelToStack,
                            //intakeController.intakeAtStackOnePixelAction(), //TODO : Determine which to use
                            intakeController.intakeAtStackUsingMagazineSensorAction(0.8),//1
                            //TODO: CHANGE ACTION TO Rotate once each of Horizontal intake, rather than continuous rotation.
                            // Intake should be running more, less revolutions of horizontal intake.

                            new SleepAction(0.5),
                            intakeController.intakeReverseAction(0.7),
                            intakeController.intakeLiftUpAction(),
                            trajStackToAfterStack,
                            trajAfterStackToDropYellowPixel,
                            outtakeController.moveReadyForTransferToTransferAction(),
                            outtakeController.moveTransferToPickupAction(),
                            outtakeController.movePickupToReadyForTransferAction(),
                            outtakeController.moveReadyForTransferToDropAction(OuttakeSlides.OUTTAKE_SLIDE_STATE.DROP_LOWEST),
                            new SleepAction(0.7),
                            outtakeController.dropTwoPixelAction(),
                            new SleepAction(0.1),
                            intakeController.intakeLiftUpAction(),
                            parkingArm.extendParkingArmAction(),
                            trajDropYellowPixelToPark,
                            outtakeController.moveOuttakeToEndStateAction()
                    )
            );
        }

    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        //telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode:");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Select Starting Position using XYAB on Logitech (or ▢ΔOX on Playstation) on gamepad 1:", "");
            telemetry.addData("    Red Left    ", "(B / O)");
            telemetry.addData("    Blue Right ", "(Y / Δ)");
            if (gamepadController.gp1GetCirclePress()) {
                GameField.startPosition = GameField.START_POSITION.RED_LEFT;
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                break;
            }
            if (gamepadController.gp1GetTrianglePress()) {
                GameField.startPosition = GameField.START_POSITION.BLUE_RIGHT;
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
            telemetry.addData("    Purple and Park                 ", "(B / O)");
            telemetry.addData("    Purple, Stack1, Yellow and Park ", "(X / ▢)");

            if (gamepadController.gp1GetCirclePress()) {
                autoOption = AUTO_OPTION.PURPLE_AND_PARK;
                break;
            }

            if (gamepadController.gp1GetSquarePress()) {
                autoOption = AUTO_OPTION.PRELOAD_STACK1_AND_PARK;
                break;
            }

            telemetry.update();
        }

        while (!isStopRequested()) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", GameField.startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            telemetry.addLine("Select Pathway and Parking option");
            telemetry.addData("    Wall Rigging, Corner parking", "Y / Δ");
            telemetry.addData("    Wall Rigging, Front of Corner parking", "X / ▢");
            telemetry.addData("    Stage Door. Middle parking", "B / O ");
            telemetry.addData("    Stage Door. Front of Middle parking", "A / X ");

            if (gamepadController.gp1GetTrianglePress()) {
                pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
                parkingOption = PARKING_OPTION.WALL_RIGGING;
                break;
            }

            if (gamepadController.gp1GetSquarePress()) {
                pathwayOption = PATHWAY_OPTION.RIGGING_WALL;
                parkingOption = PARKING_OPTION.WALL_RIGGING_FRONT;
                break;
            }

            if (gamepadController.gp1GetCirclePress()) {
                pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                parkingOption = PARKING_OPTION.STAGEDOOR;
                break;
            }

            if (gamepadController.gp1GetCrossPress()) {
                pathwayOption = PATHWAY_OPTION.STAGEDOOR;
                parkingOption = PARKING_OPTION.STAGEDOOR_FRONT;
                break;
            }
            telemetry.update();
        }

        if (autoOption == AUTO_OPTION.PRELOAD_STACK1_AND_PARK) {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", GameField.startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addData("Selected Pathway and Parking option", pathwayOption);
                telemetry.addLine("Drop Stack pixels on");
                telemetry.addData("    Back Drop ", "Y / Δ");
                telemetry.addData("    Back Stage", "B / O ");
                if (gamepadController.gp1GetTrianglePress()) {
                    dropStackPixelOption = DROP_STACK_PIXEL_OPTION.BACK_DROP;
                    break;
                }
                if (gamepadController.gp1GetCirclePress()) {
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

    public Pose2d adjustPos(Pose2d pose, double deltaX, double deltaY) {
        return pose = new Pose2d(pose.position.x + deltaX,pose.position.y + deltaY, pose.heading.log());
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

        parkingArm = new ParkingArm(hardwareMap, telemetry);
        telemetry.addLine("ParkingArm Initialized");
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
