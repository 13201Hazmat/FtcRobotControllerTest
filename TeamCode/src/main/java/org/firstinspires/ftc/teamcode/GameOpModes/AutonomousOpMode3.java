package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.AutonomousController;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.BlinkinDisplay;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Elevator;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.MajorArm;
import org.firstinspires.ftc.teamcode.SubSystems.MinorArm;
import org.firstinspires.ftc.teamcode.SubSystems.Spinner;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;


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
@Autonomous(name = "Autonomous 3", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class AutonomousOpMode3 extends LinearOpMode {

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
    public BlinkinDisplay blinkinDisplay;

    public Vision vision;
    public Pose2d startPose = GameField.BLUE_WAREHOUSE_STARTPOS;
    public GameField.PARKING_LOCATION parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
    public GameField.AUTONOMOUS_ROUTE autonomousRoute = GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER;

    boolean parked = false ;
    boolean autonomousStarted = false;

    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;
    public GameField.VISION_IDENTIFIED_TARGET targetZone = GameField.VISION_IDENTIFIED_TARGET.LEVEL1;
    int targetZoneLevel = 0;

    double af = GameField.ALLIANCE_FACTOR;

    Trajectory traj;
    TrajectorySequence trajSeq;

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
        blinkinDisplay = new BlinkinDisplay(hardwareMap);


        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intake, elevator,
                magazine, spinner, majorArm, minorArm, blinkinDisplay);
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
        //autonomousController.moveAutoElevatorLevel1();
        //autonomousController.moveAutoMagazineToTransport();
        majorArm.moveMajorArmWristToInitPosition();
        autonomousController.runAutoControl();

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {
            //Run Vuforia Tensor Flow
            targetZone = vision.runVuforiaTensorFlow();
            targetZoneLevel = targetZone.ordinal()+1;

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

                autonomousController.moveAutoElevatorLevel1();
                autonomousController.moveAutoMagazineToTransport();

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

        autonomousController.moveAutoElevatorLevel0();
        safeWait(100);
        GameField.currentPose = driveTrain.getPoseEstimate();
        GameField.poseSetInAutonomous = true;
    }

    TrajectorySequence trajInitToOffWall;
    TrajectorySequence trajOffWallToBarCode[] = new TrajectorySequence[0];
    TrajectorySequence trajASBarCodeToCarousal[] = new TrajectorySequence[0];
    TrajectorySequence trajASCarousalToAllianceShippingHub[] = new TrajectorySequence[0];
    TrajectorySequence trajASAllianceShippingHubToParking;



    public void buildAutoStorage(){
        Pose2d  initPose,
                offWallPose,
                barcodePose[] = new Pose2d[0],
                carousalPose,
                allianceShippingHubPose,
                storageParkingPose,
                warehouseMidAlongWallParkingPose1,
                warehouseMidAlongWallParkingPose2,
                warehouseAlongWallParkingPose,
                warehouseMidThroughBarrierParkingPose,
                warehouseThroughBarrierParkingPose,
                warehouseSharedShippingHubParkingPose;

        //Move from init to offWallPosition
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            //BLUE_STORAGE_STARTPOS =  Pose2d(-61,-40,Math.toRadians(180));
            initPose = GameField.BLUE_STORAGE_STARTPOS;
            offWallPose = new Pose2d(-55,-40,Math.toRadians(180));
            barcodePose[1] = new Pose2d(-46, -32, Math.toRadians(-170));
            barcodePose[2] = new Pose2d(-48, -40, Math.toRadians(180));
            barcodePose[3] = new Pose2d(-47, -47, Math.toRadians(165));
            carousalPose = new Pose2d(-60, -66, Math.toRadians(-155));
            allianceShippingHubPose = new Pose2d(-60, -66, Math.toRadians(-155));
            storageParkingPose = new Pose2d(-39,-69, Math.toRadians(90));
            warehouseMidAlongWallParkingPose1 = new Pose2d(-69, -23, Math.toRadians(90));
            warehouseMidAlongWallParkingPose2 = new Pose2d(-69, 55, Math.toRadians(90));
            warehouseAlongWallParkingPose = new Pose2d(-69, 55, Math.toRadians(90));
            warehouseMidThroughBarrierParkingPose = new Pose2d(-52, -40, Math.toRadians(60));
            warehouseThroughBarrierParkingPose = new Pose2d(-46, 60, Math.toRadians(90));
            warehouseSharedShippingHubParkingPose = new Pose2d(-46, 68, Math.toRadians(0));
        } else {
            //RED_STORAGE_STARTPOS =  Pose2d(61,-40,Math.toRadians(0));
            initPose = GameField.RED_STORAGE_STARTPOS;
            offWallPose = new Pose2d(55,-40,Math.toRadians(0));
            barcodePose[1] = new Pose2d(50,-38, Math.toRadians(-5));
            barcodePose[2] = new Pose2d(48,-37, Math.toRadians(-30));
            barcodePose[3] = new Pose2d(47.5, -30, Math.toRadians(-45));
            carousalPose = new Pose2d(51, -65, Math.toRadians(-60));
            allianceShippingHubPose = new Pose2d(33.5, -23.5, Math.toRadians(-45));
            storageParkingPose = new Pose2d(36, -69, Math.toRadians(90));
            warehouseMidAlongWallParkingPose1 = new Pose2d(65, -23, Math.toRadians(90));
            warehouseMidAlongWallParkingPose2 = new Pose2d(65, 50, Math.toRadians(90));
            warehouseAlongWallParkingPose = new Pose2d(55, 50, Math.toRadians(150));
            warehouseMidThroughBarrierParkingPose = new Pose2d(52, -40, Math.toRadians(120));
            warehouseThroughBarrierParkingPose = new Pose2d(46, 60, Math.toRadians(90));
            warehouseSharedShippingHubParkingPose = new Pose2d(46, 68, Math.toRadians(180));;
        }

        trajInitToOffWall = driveTrain.trajectorySequenceBuilder(initPose)
                    .lineToLinearHeading(offWallPose)
                    .build();

        for (int i=1; i<=3; i++) {
            trajOffWallToBarCode[i] = driveTrain.trajectorySequenceBuilder(offWallPose)
                    .setVelConstraint(driveTrain.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                    //.lineToLinearHeading(offWallPosition)
                    .lineToLinearHeading(barcodePose[i])
                    .resetVelConstraint()
                    .build();
            trajASBarCodeToCarousal[i] = driveTrain.trajectorySequenceBuilder(barcodePose[i])
                    .setVelConstraint(driveTrain.getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(carousalPose)
                    .resetVelConstraint()
                    .build();
            int finalI = i;
            trajASCarousalToAllianceShippingHub[i] = driveTrain.trajectorySequenceBuilder(carousalPose)
                    .addTemporalMarker(1, ()->{moveElevatorToLevel(finalI);})
                    .lineToLinearHeading(allianceShippingHubPose)
                    .build();
        }

        if (parkingLocation == GameField.PARKING_LOCATION.STORAGE) {
            trajASAllianceShippingHubToParking = driveTrain.trajectorySequenceBuilder(allianceShippingHubPose)
                    .addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                    .lineToLinearHeading(storageParkingPose)
                    .build();
        } else { //parkingLocation == GameField.PARKING_LOCATION.WAREHOUSE
            if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
                if (GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                    trajASAllianceShippingHubToParking = driveTrain.trajectorySequenceBuilder(allianceShippingHubPose)
                            .addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                            .splineTo(warehouseMidAlongWallParkingPose1.vec(),warehouseMidAlongWallParkingPose1.getHeading())
                            .splineTo(warehouseMidAlongWallParkingPose2.vec(),warehouseMidAlongWallParkingPose2.getHeading())
                            .splineTo(warehouseSharedShippingHubParkingPose.vec(),warehouseSharedShippingHubParkingPose.getHeading())
                            .build();
                } else { // ! GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                    trajASAllianceShippingHubToParking = driveTrain.trajectorySequenceBuilder(allianceShippingHubPose)
                            .addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                            .splineTo(warehouseMidAlongWallParkingPose1.vec(),warehouseMidAlongWallParkingPose1.getHeading())
                            .splineTo(warehouseMidAlongWallParkingPose2.vec(),warehouseMidAlongWallParkingPose2.getHeading())
                            .splineTo(warehouseAlongWallParkingPose.vec(),warehouseAlongWallParkingPose.getHeading())
                            .build();
                }
            } else { //(autonomousRoute == GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER)
                if (GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                    trajASAllianceShippingHubToParking = driveTrain.trajectorySequenceBuilder(allianceShippingHubPose)
                            .addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                            .splineTo(warehouseMidThroughBarrierParkingPose.vec(),warehouseMidThroughBarrierParkingPose.getHeading())
                            .splineTo(warehouseThroughBarrierParkingPose.vec(),warehouseThroughBarrierParkingPose.getHeading())
                            .splineTo(warehouseSharedShippingHubParkingPose.vec(),warehouseSharedShippingHubParkingPose.getHeading())
                            .build();
                } else { // ! GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                    trajASAllianceShippingHubToParking = driveTrain.trajectorySequenceBuilder(allianceShippingHubPose)
                            .addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                            .splineTo(warehouseMidThroughBarrierParkingPose.vec(),warehouseMidThroughBarrierParkingPose.getHeading())
                            .splineTo(warehouseThroughBarrierParkingPose.vec(),warehouseThroughBarrierParkingPose.getHeading())
                            .build();
                }
            }

        }
    }

    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoStorage(){
        //Logic for waiting
        safeWait(100);
        driveTrain.followTrajectorySequence(trajInitToOffWall);
        safeWait(1000);

        //Move arm to Pickup Capstone level and open Grip
        moveMajorArmToPickupAndOpenClaw();
        safeWait(2000);

        //Move forward to Capstone Pickup Position
        driveTrain.followTrajectorySequence(trajOffWallToBarCode[targetZoneLevel]);

        //Collect Capstone and move arm to parking position
        moveMajorArmToParkingAfterClosingClaw();
        safeWait(1000);

        //Move forward to Capstone Pickup Position
        driveTrain.followTrajectorySequence(trajASBarCodeToCarousal[targetZoneLevel]);

        //Rotate Carousal
        rotateCarousal();

        //Move To Shipping Unit
        driveTrain.followTrajectorySequence(trajASCarousalToAllianceShippingHub[targetZoneLevel]);

        //Drop pre-loaded box in correct level
        dropBoxToLevel();
        safeWait(500);

        //Move and park completely in Warehouse
        driveTrain.followTrajectorySequence(trajASAllianceShippingHubToParking);

        moveElevatorToLevel(0);
        safeWait(1000);
        return;
    }


    public void runAutoWarehouse(){
        //public static final Pose2d BLUE_WAREHOUSE_STARTPOS =  new Pose2d(-61,7,Math.toRadians(180));
        //public static final Pose2d RED_WAREHOUSE_STARTPOS =  new Pose2d(61,7,Math.toRadians(0));

        safeWait(100);


        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-56,7,Math.toRadians(180)))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(56,7,Math.toRadians(0)))
                    .build();
        }
        driveTrain.followTrajectory(traj);
        safeWait(1000);

        //Move arm to Pickup Capstone level and open Grip
        moveMajorArmToPickupAndOpenClaw();

        safeWait(1000);

        //Move forward to Capstone Pickup Position
        switch (targetZone) {
            case LEVEL1:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-46, 17, Math.toRadians(-170)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(50, 9, Math.toRadians(-5)))
                            .build();
                }
                break;
            case LEVEL2:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-48, 9, Math.toRadians(180)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(48, 8, Math.toRadians(-30)))
                            .build();
                }
                break;
            case LEVEL3:
            case UNKNOWN:
                if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(-47, -1, Math.toRadians(165)))
                            .build();
                } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                    traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                            .lineToLinearHeading(new Pose2d(50, 15, Math.toRadians(-45)))
                            .build();
                }
                break;
        }
        driveTrain.followTrajectory(traj);

        //Collect Capstone and move arm to parking position
        moveMajorArmToParkingAfterClosingClaw();
        safeWait(1000);

        //Move to shipping unit and place the pre-loaded freight
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE){
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(-33, 0 , Math.toRadians(135)))
                    .build();
        } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
            traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                    .lineToLinearHeading(new Pose2d(32, -3, Math.toRadians(45)))
                    .build();
        }
        driveTrain.followTrajectory(traj);
        //Drop the Box
        dropBoxToLevel();
        safeWait(100);

        autonomousController.moveAutoMagazineToTransport();
        autonomousController.moveAutoElevatorLevel1();

        //Move and park completely in Warehouse
        if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-70, 8, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-70, 55, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
            } else { //(GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE)
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(68, 8, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(68, 50, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(55, 50, Math.toRadians(150)))
                        .build();
                driveTrain.followTrajectory(traj);
            }
        } else { //(autonomousRoute == GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER)
            if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-48, -10, Math.toRadians(60)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(-42, 60, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
            } else {
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(48, -10, Math.toRadians(120)))
                        .build();
                driveTrain.followTrajectory(traj);
                traj = driveTrain.trajectoryBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(new Pose2d(42, 60, Math.toRadians(90)))
                        .build();
                driveTrain.followTrajectory(traj);
            }
        }


        // Run Intake and Move Elevator to Level
        //runIntakeToCollect();
        safeWait(200);
        autonomousController.moveAutoElevatorLevel0();
        safeWait(1000);

    }

    public void rotateCarousal() {
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.CLOCKWISE;
        } else {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.ANTICLOCKWISE;
        }
        autonomousController.runAutoControl();
        safeWait(3500);
        autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.STOPPED;
        autonomousController.runAutoControl();
    }

    //Drops pre-loaded box at the correct level determined by capstone position
    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void dropBoxToLevel(){
        autonomousController.moveAutoMagazineToDrop();
        safeWait(1000);
        autonomousController.moveAutoMagazineToCollect();
        safeWait(1000);

        /*switch(targetZone) {
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
        }*/

    }

    //TODO: Update code to use autoController instead of accessing the subsystems directly
    public void dropFreightLevel1(){
        //autonomousController.moveAutoElevatorLevel1();
        //safeWait(1000);
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

    public void moveElevatorToLevel(int level){
        switch (level) {
            case 0 : autonomousController.moveAutoElevatorLevel0(); break;
            case 1 : autonomousController.moveAutoElevatorLevel1(); break;
            case 2 : autonomousController.moveAutoElevatorLevel2(); break;
            case 3 : autonomousController.moveAutoElevatorLevel3(); break;
        }
    }


    /**
     * Hybrid Commands For Autonomous OpMode
     */

    public void moveMajorArmToParkingAfterClosingClaw(){
        safeWait(1000);
        autonomousController.closeAutoMajorClaw();
        safeWait(500);
        autonomousController.moveAutoMajorArmCapstone();
        safeWait(500);
        majorArm.moveMajorArmWristToParkedPosition();
        safeWait(500);
        autonomousController.moveAutoMajorArmPark();
        safeWait(500);

    }


    public void moveMajorArmToPickupAndOpenClaw(){
        majorArm.moveMajorArmWristToPickupPosition();
        safeWait(500);
        autonomousController.moveAutoMajorArmPickup();
        safeWait(500);
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
            driveTrain.update();
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
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                break;
            }
            if (gamepadController.gp1GetButtonXPress()) {
                GameField.playingAlliance = GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE;
                GameField.ALLIANCE_FACTOR = 1;
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                break;
            }
            telemetry.update();
        }
        telemetry.update();
        safeWait(200);

        //***** Select Start Pose ******


        while (!isStopRequested()) {
            telemetry.addData("Enter Start Pose :","");
            telemetry.addData("  STORAGE : (Y)","");
            telemetry.addData("  WAREHOUSE : (A)","");
            telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
            if (gamepadController.gp1GetButtonYPress()) {
                GameField.startPosition = GameField.START_POSITION.STORAGE;
                telemetry.addData("StartPose : ", GameField.startPosition);
                break;
            }
            if (gamepadController.gp1GetButtonAPress()) {
                GameField.startPosition = GameField.START_POSITION.WAREHOUSE;
                telemetry.addData("StartPose : ", GameField.startPosition);
                break;
            }
            telemetry.update();
        }

        telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
        telemetry.addData("StartPose : ", GameField.startPosition);

        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.RED_ALLIANCE) {
            if (GameField.startPosition == GameField.START_POSITION.STORAGE) {
                startPose = GameField.RED_STORAGE_STARTPOS;
            } else { // StartPosition == WAREHOUSE
                startPose = GameField.RED_WAREHOUSE_STARTPOS;
            }
        } else { // Alliance == BLUE
            if (GameField.startPosition == GameField.START_POSITION.STORAGE) {
                startPose = GameField.BLUE_STORAGE_STARTPOS;
            } else { // StartPosition == WAREHOUSE
                startPose = GameField.BLUE_WAREHOUSE_STARTPOS;
            }
        }


        if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
            parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Parking Location: ",parkingLocation);
                telemetry.addData("Enter Autonomous Route :","");
                telemetry.addData("  WAREHOUSE PARK ALONG WALL (X)","");
                telemetry.addData("  WAREHOUSE PARK THROUGH BARRIER (B)","");
                if (gamepadController.gp1GetButtonXPress()) {
                    autonomousRoute = GameField.AUTONOMOUS_ROUTE.ALONG_WALL;
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    autonomousRoute = GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER;
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    break;
                }
                telemetry.update();
            }
        }

        if (GameField.startPosition == GameField.START_POSITION.STORAGE) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Enter Autonomous Route & Parking :","");
                telemetry.addData("  STORAGE PARK (A)","");
                telemetry.addData("  WAREHOUSE PARK ALONG WALL (X)","");
                telemetry.addData("  WAREHOUSE PARK THROUGH BARRIER (B)","");
                if (gamepadController.gp1GetButtonAPress()) {
                    autonomousRoute = GameField.AUTONOMOUS_ROUTE.NOT_APPLICABLE;
                    parkingLocation = GameField.PARKING_LOCATION.STORAGE;
                    GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB = false;
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    telemetry.addData("Parking Location: ",parkingLocation);
                    break;
                }
                if (gamepadController.gp1GetButtonXPress()) {
                    autonomousRoute = GameField.AUTONOMOUS_ROUTE.ALONG_WALL;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    telemetry.addData("Parking Location: ",parkingLocation);
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    autonomousRoute = GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER;
                    parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    telemetry.addData("Parking Location: ",parkingLocation);
                    break;
                }
                telemetry.update();
            }
        }

        if (parkingLocation == GameField.PARKING_LOCATION.WAREHOUSE) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Autonomous Route : ", autonomousRoute);
                telemetry.addData("Parking Location: ", parkingLocation);
                telemetry.addData("Enter End Parking facing Shared Shipping Hub :", "");
                telemetry.addData("  YES (Y)", "");
                telemetry.addData("  NO (A)", "");
                if (gamepadController.gp1GetButtonYPress()) {
                    GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB = true;
                    telemetry.addData("End Parking facing Shared Shipping Hub :", GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB);
                    break;
                }
                if (gamepadController.gp1GetButtonAPress()) {
                    GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB = false;
                    telemetry.addData("End Parking facing Shared Shipping Hub :", GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB);
                    break;
                }
                telemetry.update();
            }
        }

        telemetry.clearAll();
        telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
        telemetry.addData("Start Position : ", GameField.startPosition);
        telemetry.addData("Autonomous route : ", autonomousRoute);
        telemetry.addData("Parking Location : ", parkingLocation);
        telemetry.addData("End Parking facing Shared Shipping Hub :", GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB);
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

        telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
        telemetry.addData("Start Position : ", GameField.startPosition);
        telemetry.addData("Autonomous route : ", autonomousRoute);
        telemetry.addData("Parking Location : ", parkingLocation);

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
        telemetry.addData("Vision targetZone :", targetZone);
        telemetry.addData("Vision targetZoneLevel :", targetZoneLevel);

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

