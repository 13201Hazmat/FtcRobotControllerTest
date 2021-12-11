package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.AutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.MinorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;


/**
 * Ultimate Goal Autonomous mode <BR>
 *
 * This code describes how Autonomous mode is done by Hazmat Robot for Ultimate Goal.<BR>
 * The following options are coded here, and selectable through Gamepad inputs to set up <BR>
 *     <emsp>Playing Alliance : Red or Blue</emsp>
 *     <emsp>Start Line : Inner or Outer</emsp>
 *     <emsp>Game options :</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch and park</emsp>
 *     <emsp>     Launch rings to High Goal or Power Shot, drop Wobble Goal, Pick rings from target marker, launch, move Wobble Goal2 and park</emsp>
 *
 * The code for Red and Blue are written as reflection of each other.<BR>
 * Camera on either side is used using Vuforia to determine target for Wobble Goal<BR>
 */
//TODO: Copy and Rename Autonomous Mode
@Autonomous(name = "Autonomous", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class AutonomousOpMode extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadController gamepadController;
    public AutonomousController autonomousController;
    public DriveTrain driveTrain;
    public Intake intake;
    public Elevator elevator;
    public Magazine magazine;
    public Spinner spinner;
    public MajorArm majorArm;
    public MinorArm minorArm;

    public Vision vision;
    public Pose2d startPose = GameField.BLUE_WAREHOUSE_STARTPOS;
    public GameField.PARKING_LOCATION parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
    public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;

    double af = GameField.ALLIANCE_FACTOR;

    Trajectory traj;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        intake = new Intake(hardwareMap);
        elevator = new Elevator(hardwareMap);
        magazine = new Magazine(hardwareMap);
        spinner = new Spinner(hardwareMap);
        majorArm = new MajorArm(hardwareMap);
        minorArm = new MinorArm(hardwareMap);

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intake, elevator, magazine, spinner, majorArm, minorArm);
        autonomousController = new AutonomousController(driveTrain,
                intake,
                elevator,
                magazine,
                spinner,
                majorArm,
                minorArm);

        //Key Pay inputs to select Game Plan;
        selectGamePlan();
        vision = new Vision(hardwareMap, activeWebcam);
        af = GameField.ALLIANCE_FACTOR;

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        driveTrain.getLocalizer().setPoseEstimate(startPose);

        //Robot starts with Elevator in Collect State, with preloaded box
        //On Init, Elevator moves to Level 1, Magazine moves to transport
        autonomousController.moveAutoElevatorLevel1();
        autonomousController.moveAutoMagazineToTransport();
        majorArm.moveMajorArmWristToInitPosition();
        autonomousController.runAutoControl();

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = vision.runVuforiaTensorFlow();

            if (!parked){
                autonomousController.runAutoControl();
            }

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {

                vision.deactivateVuforiaTensorFlow();

                // Logic to determine and run defined Autonomous mode
                if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
                    runAutoWarehouse();
                } else { //GameField.startPosition == GameField.START_POSITION.STORAGE
                    runAutoStorage();
                }


                //Move to Launching Position
                parked = true;

                //Write last position to static class to be used as initial position in TeleOp
                GameField.currentPose = driveTrain.getPoseEstimate();
                GameField.poseSetInAutonomous = true;

                if (DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }
            }

        }

        //Write last position to static class to be used as initial position in TeleOp
        GameField.currentPose = driveTrain.getPoseEstimate();
        GameField.poseSetInAutonomous = true;
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoStorage(){

        //public static final Pose2d BLUE_STORAGE_STARTPOS =  new Pose2d(-61,-31,Math.toRadians(180));
        //public static final Pose2d RED_STORAGE_STARTPOS =  new Pose2d(61,-40,Math.toRadians(0));

        //Logic for waiting
        safeWait(100);

        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-55,-31,Math.toRadians(180)))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(55,-40,Math.toRadians(0)))
                    .build();
        }

        //Move arm to Pickup Capstone level and open Grip
        moveMajorArmToPickupAndOpenClaw();
        safeWait(2000);

        //Move forward to Capstone Pickup Position
        switch (targetZone) {
            case LEVEL1:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, -33, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(49,-39, Math.toRadians(0)))
                            .build();
                }
                break;
            case LEVEL2:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, -41, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(49,-31, Math.toRadians(0)))
                            .build();
                }
                break;

            case LEVEL3:
            case UNKNOWN:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, -49, Math.toRadians(180)))
                            .build();
                    break;
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(45.5, -23, Math.toRadians(0)))
                            .build();
                }
                break;
        }
        driveTrain.followTrajectory(traj);

        //Collect Capstone and move arm to parking position
        moveMajorArmToParkingAfterClosingClaw();

        //Move To Carousal
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-65, -61, Math.toRadians(-155)))
                    .build();
        } else {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(57.5, -61.5, Math.toRadians(-60)))
                    .build();
        }
        driveTrain.followTrajectory(traj);

        //Rotate Carousal
        rotateCarousal();

        //Move To Shipping Unit
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-33.5, -21.5 , Math.toRadians(-135)))
                    .build();
        } else {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(36, -21, Math.toRadians(-45)))
                    .build();
        }
        driveTrain.followTrajectory(traj);

        //Drop pre-loaded box in correct level
        dropBoxToLevel();

        safeWait(1000);
        moveElevatorToLevel1();

        if (parkingLocation == GameField.PARKING_LOCATION.STORAGE) {
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-36,-65, Math.toRadians(90)))
                        .build();
            } else {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(36, -65, Math.toRadians(90)))
                        .build();
            }
            driveTrain.followTrajectory(traj);
        } else {
            //Move through Central Area and Park in Warehouse
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(-6, -20, Math.toRadians(90)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(-12, 12, Math.toRadians(135)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(-40, 40, Math.toRadians(135)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(-40, 64, Math.toRadians(180)), Math.toRadians(0)) //TODO: Check position
                        .build();
            } else {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineToSplineHeading(new Pose2d(6, -20, Math.toRadians(90)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(12, 12, Math.toRadians(45)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(40, 40, Math.toRadians(45)), Math.toRadians(0)) //TODO: Check position
                        .splineToSplineHeading(new Pose2d(40, 64, Math.toRadians(0)), Math.toRadians(0)) //TODO: Check position
                        .build();
            }
            driveTrain.followTrajectory(traj);
            runIntakeToCollect();
            safeWait(2000);
            moveElevatorToCollect();
        }
    }


    public void runAutoWarehouse(){
        //public static final Pose2d BLUE_WAREHOUSE_STARTPOS =  new Pose2d(-61,14,Math.toRadians(180));
        //public static final Pose2d RED_WAREHOUSE_STARTPOS =  new Pose2d(61,7,Math.toRadians(0));

        safeWait(100);

        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-55,-14,Math.toRadians(180)))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(55,7,Math.toRadians(0)))
                    .build();
        }

        //Move arm to Pickup Capstone level and open Grip
        moveMajorArmToPickupAndOpenClaw();

        safeWait(1000);

        //Move forward to Capstone Pickup Position
        switch (targetZone) {
            case LEVEL1:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, -14, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(49, 9, Math.toRadians(180)))
                            .build();
                }
                break;
            case LEVEL2:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, 6, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(48, 11, Math.toRadians(-30)))
                            .build();
                }
                break;
            case LEVEL3:
            case UNKNOWN:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-49, 11, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(47, 13, Math.toRadians(-60)))
                            .build();
                }
                break;
        }
        driveTrain.followTrajectory(traj);

        //Collect Capstone and move arm to parking position
        moveMajorArmToParkingAfterClosingClaw();

        //Move to shipping unit and place the pre-loaded freight
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-42, 12 , Math.toRadians(-135)))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(47, 13, Math.toRadians(-60)))
                    .build();
        }
        driveTrain.followTrajectory(traj);
        //Drop the Box
        dropBoxToLevel();
        safeWait(100);

        for (int looper = 0; looper < 3; looper++){
            //Move Intake to collect mode
            autonomousController.moveAutoMagazineToTransport();
            autonomousController.moveAutoElevatorLevel0();

            // Move into Warehouse from Shipping Unit
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                        .splineTo(new Vector2d(-60, 42), Math.toRadians(90))
                        .build();
            } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineTo(new Vector2d(60, 12), Math.toRadians(90))
                        .splineTo(new Vector2d(60, 42), Math.toRadians(90))                        .build();
            }
            //Pick Up Freight
            runIntakeToCollect();
            safeWait(1000);
            moveElevatorToLevel1();

            //Move To Shipping Unit from Warehouse
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineTo(new Vector2d(-60, 12), Math.toRadians(90))
                        .splineTo(new Vector2d(-42, 42), Math.toRadians(90))
                        .build();
            } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .splineTo(new Vector2d(60, 12), Math.toRadians(90))
                        .splineTo(new Vector2d(42, 42), Math.toRadians(90))
                        .build();
            }
            driveTrain.followTrajectory(traj);

            dropFreightLevel1();
        }

        //Move and park completely in Warehouse
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .splineTo(new Vector2d(-42, 42), Math.toRadians(0))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .splineTo(new Vector2d(36, 60), Math.toRadians(0))
                    .build();
        }
        driveTrain.followTrajectory(traj);

        // Run Intake and Move Elevator to Level 1
        runIntakeToCollect();
        safeWait(2000);
        moveElevatorToLevel0();

    }

    public void rotateCarousal() {
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.CLOCKWISE;
        } else {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.ANTICLOCKWISE;
        }
        autonomousController.runAutoControl();
        safeWait(4000);
        autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.STOPPED;
        autonomousController.runAutoControl();
    }

    //Drops pre-loaded box at the correct level determined by capstone position
    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void dropBoxToLevel(){
        switch(targetZone) {
            case LEVEL1:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 1
                dropFreightLevel1();
                break;
            case LEVEL2:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 2
                autonomousController.moveAutoElevatorLevel2();
                safeWait(1000);
                autonomousController.moveAutoMagazineToDrop();
                safeWait(1000);
                autonomousController.moveAutoMagazineToCollect();
                safeWait(1000);
                break;
            case LEVEL3:
                //If Capstone on Level 1, Drops Pre-Loaded Box on Level 3
                autonomousController.moveAutoElevatorLevel3();
                safeWait(1000);
                autonomousController.moveAutoMagazineToDrop();
                safeWait(1000);
                autonomousController.moveAutoMagazineToCollect();
                safeWait(1000);
                break;
        }

    }

    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void dropFreightLevel1(){
        autonomousController.moveAutoElevatorLevel1();
        safeWait(1000);
        autonomousController.moveAutoMagazineToDrop();
        safeWait(1000);
        autonomousController.moveAutoMagazineToCollect();
        safeWait(1000);
    }

    //Runs the Intake and moves the Elevator to Level1
    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void runIntakeToCollect(){
        autonomousController.moveAutoElevatorLevel0();
        autonomousController.startAutoIntake();
    }

    public void moveElevatorToLevel1(){
        autonomousController.moveAutoElevatorLevel1();
    }

    public void moveElevatorToLevel0(){
        autonomousController.moveAutoElevatorLevel0();
    }

    public void moveElevatorToCollect(){
        autonomousController.moveAutoElevatorLevel0();
    }


    /**
     * Hybrid Commands For Autonomous OpMode
     */

    public void moveMajorArmToParkingAfterClosingClaw(){
        autonomousController.closeAutoMajorClaw();
        safeWait(1000);
        autonomousController.moveAutoMajorArmPark();
        majorArm.moveMajorArmWristToPosition();
        safeWait(1000);
    }


    public void moveMajorArmToPickupAndOpenClaw(){
        autonomousController.moveAutoMajorArmPickup();
        safeWait(1000);
        autonomousController.openAutoMajorClaw();
    }

    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            autonomousController.runAutoControl();
        }
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "4:47 :: 2/13");

        //***** Select Alliance ******
        telemetry.addData("Enter PLaying Alliance :", "(Blue: (X),    Red: (B))");
        telemetry.update();

        //Add logic to select autonomous mode based on keypad entry
        while (!isStopRequested()) {
            if (gamepadController.gp1GetButtonBPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.RED_ALLIANCE;
                GameField.ALLIANCE_FACTOR = -1;
                telemetry.addData("Playing Alliance Selected : ", "RED_ALLIANCE");
                break;
            }
            if (gamepadController.gp1GetButtonXPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                GameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", "BLUE_ALLIANCE");
                break;
            }
            telemetry.update();
        }
        telemetry.update();
        safeWait(200);

        //***** Select Start Pose ******
        telemetry.addData("Enter Start Pose :","");
        telemetry.addData("  WAREHOUSE: (A)","");
        telemetry.addData("  STORAGE PARK STORAGE: (Y)","");
        telemetry.addData("  STORAGE PARK WAREHOUSE: (A)","");

        while (!isStopRequested()) {
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
                if (gamepadController.gp1GetButtonAPress()) {
                    GameField.startPosition = GameField.START_POSITION.WAREHOUSE;
                    startPose = GameField.RED_WAREHOUSE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Start Pose : ", "RED_WAREHOUSE");
                    break;
                }
                if (gamepadController.gp1GetButtonYPress()) {
                    GameField.startPosition = GameField.START_POSITION.STORAGE;
                    startPose = GameField.RED_STORAGE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.STORAGE;
                    telemetry.addData("Start Pose : ", "RED_STORAGE_PARK_STORAGE");
                    break;
                }
                if(gamepadController.gp1GetButtonAPress()){
                    GameField.startPosition = GameField.START_POSITION.STORAGE;
                    startPose = GameField.RED_STORAGE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Start Pose : ", "RED_STORAGE_PARK_WAREHOUSE");
                    break;
                }
            }
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                if (gamepadController.gp1GetButtonAPress()) {
                    GameField.startPosition = GameField.START_POSITION.WAREHOUSE;
                    startPose = GameField.BLUE_WAREHOUSE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Start Pose : ", "BLUE_WAREHOUSE");
                    break;
                }
                if (gamepadController.gp1GetButtonYPress()) {
                    GameField.startPosition = GameField.START_POSITION.STORAGE;
                    startPose = GameField.BLUE_STORAGE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.STORAGE;
                    telemetry.addData("Start Pose : ", "BLUE_STORAGES_PARK_STORAGE");
                    break;
                }
                if(gamepadController.gp1GetButtonAPress()){
                    GameField.startPosition = GameField.START_POSITION.STORAGE;
                    startPose = GameField.BLUE_STORAGE_STARTPOS;
                    activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Start Pose : ", "BLUE_STORAGE_PARK_WAREHOUSE");
                    break;
                }
            }
            telemetry.update();
        }

        telemetry.update();
        safeWait(200);
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        telemetry.addData("GameField.playingAlliance : ", GameField.playingAlliance);
        telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
        telemetry.addData("GameField.currentPose : ", GameField.currentPose);
        telemetry.addData("startPose : ", startPose);

        //****** Drive debug ******
        telemetry.addData("Drive Mode : ", driveTrain.driveMode);
        telemetry.addData("PoseEstimate :", driveTrain.poseEstimate);
        telemetry.addData("Battery Power", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("Vision targetLevelDetected : ", vision.targetLevelDetected);
        telemetry.addData("Vision detectedLabel", vision.detectedLabel);
        telemetry.addData("Vision detectedLabelLeft :", vision.detectedLabelLeft);

        telemetry.addData("Major Arm Position : ",majorArm.getMajorArmPosition());
        telemetry.addData("Major Claw State : ",majorArm.getMajorClawState());
        telemetry.addData("Major Arm Position Count : ", majorArm.getMajorArmPositionCount());
        telemetry.addData("Major Wrist Position : ", majorArm.majorWristServo.getPosition());

        telemetry.addData("Intake State : ", intake.getIntakeMotorState());
        telemetry.addData("Intake Motor Power : ", intake.getIntakeMotorPower());

        telemetry.addData("Elevator State : ", elevator.getElevatorState());
        telemetry.addData("Elevator Position Count : ", elevator.getElevatorPositionCount());

        telemetry.addData("Magazine State : ", magazine.getMagazineServoState());
        telemetry.addData("Magazine Color Sensor State : ", magazine.getMagazineColorSensorState());
        telemetry.addData("Magazine Color Sensor Distance :",magazine.getMagazineColorSensorDistance());

        telemetry.addData("Spinner State : ", spinner.getSpinnerMotorState());
        telemetry.addData("Spinner Motor Power : ", spinner.getSpinnerMotorPower());

        telemetry.addData("Minor Arm Position : ",minorArm.getMinorServoState());
        telemetry.addData("Minor Claw State : ",minorArm.getMinorClawState());

        telemetry.update();

    }
}

