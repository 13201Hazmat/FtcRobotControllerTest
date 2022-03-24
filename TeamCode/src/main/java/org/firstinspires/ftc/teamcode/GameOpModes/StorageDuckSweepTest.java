package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

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
@Autonomous(name = "StorageDuckSweepTest", group = "00-Autonomous" , preselectTeleOp = "TeleOp")
public class StorageDuckSweepTest extends LinearOpMode {

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
    public int loopsFromWarehouseToAlShippingHub = 0;
    public boolean whLoopParkThroughBarrier = false;
    public boolean pickShippingElement = false;

    boolean parked = false ;
    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);;

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

        if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
            buildAutoWarehouse();
        } else { //GameField.startPosition == GameField.START_POSITION.STORAGE
            buildAutoStorage();
        }
        driveTrain.getLocalizer().setPoseEstimate(startPose);

        //Robot starts with Elevator in Collect State, with preloaded box
        //On Init, Elevator moves to Level 1, Magazine moves to transport
        //autonomousController.moveAutoElevatorLevel1();
        //autonomousController.moveAutoMagazineToTransport();
        majorArm.moveMajorArmWristToInitPosition();
        autonomousController.runAutoControl();
        blinkinDisplay.setPatternWhite();

        telemetry.addData("Waiting for start to be pressed.","Robot is ready!");
        telemetry.update();

        if (isStopRequested()) return;

        while (!isStopRequested()) {

            //Run Vuforia Tensor Flow
            targetZone = vision.runVuforiaTensorFlow();
            targetZoneLevel = targetZone.ordinal();

            if (!parked){
                autonomousController.runAutoControl();
            }

            if (DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            //Game Play is pressed
            while (opModeIsActive() && !isStopRequested() && !parked) {
                gameTimer.reset();

                vision.deactivateVuforiaTensorFlow();
                blinkinDisplay.setPatternDemo();

                autonomousController.moveAutoElevatorLevel1();
                autonomousController.moveAutoMagazineToTransport();
                intake.intakeMotorPower1 = 1; //setting to 90% to avoid picking 2

                // Logic to determine and run defined Autonomous mode
                if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
                    runAutoWarehouse();
                } else { //GameField.startPosition == GameField.START_POSITION.STORAGE
                    runAutoStorage();
                }


                //Move to Launching Position
                parked = true;
                blinkinDisplay.setPatternBlack();

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
    TrajectorySequence trajOffWalltoCarousal;
    TrajectorySequence trajOffWalltoAlShipping;
    TrajectorySequence[] trajOffWallToBarCode = new TrajectorySequence[3];
    TrajectorySequence[] trajASBarCodeToCarousal = new TrajectorySequence[3];
    TrajectorySequence[] trajAWBarCodeToAlShipping = new TrajectorySequence[3];
    TrajectorySequence trajASAlShippingToCarousalPickDuckToAllianceShipping;
    TrajectorySequence trajASCarousalToAlShipping;
    TrajectorySequence trajASAlShippingToStorageParking;
    TrajectorySequence[] trajAlShippingToWHParking = new TrajectorySequence[2];
    TrajectorySequence[] trajWarehouseAllianceShippingLoopPick = new TrajectorySequence[4];;
    TrajectorySequence[] trajWarehouseAllianceShippingLoopDrop = new TrajectorySequence[4];;
    TrajectorySequence trajWarehouseAllianceShippingToParkBarrier;

    Pose2d  initPose;
    Pose2d offWallPose;
    Pose2d[] barcodePose = new Pose2d[3];
    Pose2d carousalPose;
    Pose2d[] pickDuckPose = new Pose2d[2];
    Pose2d carousalToAlliancePathPose;
    Pose2d alShippingHubPose;
    Pose2d storageParkingPose;
    Pose2d[] whAlongWallParkingPose = new Pose2d[3];
    Pose2d[] whAlongWallShShippingParkingPose = new Pose2d[3];;
    Pose2d[] whThroughBarrierParkingPose = new Pose2d[2];
    //Pose2d whThroughBarrierShShippingIntermidPose;
    Pose2d whThroughBarrierShShippingParkingPose;
    Pose2d[] warehousePickElementPose = new Pose2d[3];
    Pose2d[] warehouseAllianceShippingPathPose = new Pose2d[3];
    Pose2d allianceShippingHubDropElementPose;



    public void buildAutoStorage(){
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            //BLUE_STORAGE_STARTPOS =  Pose2d(-61,-40,Math.toRadians(180));
            initPose = GameField.BLUE_STORAGE_STARTPOS;
            offWallPose = new Pose2d(-56,-40,Math.toRadians(180));
            barcodePose[1-1] = new Pose2d(-48, -30, Math.toRadians(-170));
            barcodePose[2-1] = new Pose2d(-50, -38, Math.toRadians(180));
            barcodePose[3-1] = new Pose2d(-50, -48, Math.toRadians(165));

            carousalPose = new Pose2d(-58, -68, Math.toRadians(-155));

            pickDuckPose[0] = new Pose2d(-61, -62, Math.toRadians(190));
            pickDuckPose[1] = new Pose2d(-63, -61, Math.toRadians(120)); //x=-62

            carousalToAlliancePathPose = new Pose2d(-23, -63, Math.toRadians(-90)); //-19 for x
            //alShippingHubPose = new Pose2d(-38, -21 , Math.toRadians(-145));

            alShippingHubPose = new Pose2d(-24, -40, Math.toRadians(-90)); //-34 for x

            storageParkingPose = new Pose2d(-38,-70, Math.toRadians(90)); //x=-34

            whAlongWallParkingPose[0] = new Pose2d(-63, -43, Math.toRadians(90)); //x -69
            whAlongWallParkingPose[1] = new Pose2d(-72, 36, Math.toRadians(90)); //x -70
            whAlongWallParkingPose[2] = new Pose2d(-72, 45, Math.toRadians(90)); //x=-70

            warehousePickElementPose[0] = new Pose2d(-72, 46, Math.toRadians(90));
            warehousePickElementPose[1] = new Pose2d(-73, 47, Math.toRadians(90));
            warehousePickElementPose[2] = new Pose2d(-74, 48, Math.toRadians(90));
            warehouseAllianceShippingPathPose[0] = new Pose2d(-72, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[1] = new Pose2d(-73, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[2] = new Pose2d(-74, 3, Math.toRadians(90));

            allianceShippingHubDropElementPose = new Pose2d(-33, 0 , Math.toRadians(135));

            whAlongWallShShippingParkingPose[0] = new Pose2d(-40, 36, Math.toRadians(45));
            whAlongWallShShippingParkingPose[1] = new Pose2d(-40, 68, Math.toRadians(0));

            whThroughBarrierParkingPose[0] = new Pose2d(-45, -60, Math.toRadians(95));
            whThroughBarrierParkingPose[1] = new Pose2d(-43, 60, Math.toRadians(90));
            whThroughBarrierShShippingParkingPose = new Pose2d(-40, 72, Math.toRadians(180));

        } else {
            //RED_STORAGE_STARTPOS =  Pose2d(61,-40,Math.toRadians(0));
            initPose = GameField.RED_STORAGE_STARTPOS;
            offWallPose = new Pose2d(56,-40,Math.toRadians(0));
            barcodePose[1-1] = new Pose2d(49,-39, Math.toRadians(-5)); //fixed 1/8/22
            barcodePose[2-1] = new Pose2d(49,-39, Math.toRadians(-35)); //fixed 1/8/22
            barcodePose[3-1] = new Pose2d(48, -34, Math.toRadians(-50)); //fixed 1/8/22

            carousalPose = new Pose2d(50, -64, Math.toRadians(-60)); // x=53;

            pickDuckPose[0] = new Pose2d(51, -62, Math.toRadians(-10)); //AADI TO UPDATE
            pickDuckPose[1] = new Pose2d(52, -61, Math.toRadians(70)); //AADI TO UPDATE

            carousalToAlliancePathPose = new Pose2d(13, -59, Math.toRadians(-90)); // x=12
            //alShippingHubPose = new Pose2d(33.5, -23.5, Math.toRadians(-45));
            alShippingHubPose = new Pose2d(13, -27, Math.toRadians(-90));
            storageParkingPose = new Pose2d(26, -67, Math.toRadians(90)); //x = 24;


            whAlongWallParkingPose[0] = new Pose2d(63, -43, Math.toRadians(90)); //x -69
            whAlongWallParkingPose[1] = new Pose2d(72, 36, Math.toRadians(90)); //x 70
            whAlongWallParkingPose[2] = new Pose2d(72, 48, Math.toRadians(150)); //x70

            warehousePickElementPose[0] = new Pose2d(70, 46, Math.toRadians(90));
            warehousePickElementPose[1] = new Pose2d(71, 47, Math.toRadians(90));
            warehousePickElementPose[2] = new Pose2d(72, 48, Math.toRadians(90));
            warehouseAllianceShippingPathPose[0] = new Pose2d(70, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[1] = new Pose2d(71, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[2] = new Pose2d(72, 3, Math.toRadians(90));

            allianceShippingHubDropElementPose = new Pose2d(32, -3, Math.toRadians(45));

            whAlongWallShShippingParkingPose[0] = new Pose2d(40, 36, Math.toRadians(135));
            whAlongWallShShippingParkingPose[1] = new Pose2d(40, 68, Math.toRadians(180));

            whThroughBarrierParkingPose[0] = new Pose2d(45, -64, Math.toRadians(100)); //x=50
            whThroughBarrierParkingPose[1] = new Pose2d(43, 60, Math.toRadians(90));
            whThroughBarrierShShippingParkingPose = new Pose2d(40, 80, Math.toRadians(0));
        }

        //Move from init to offWallPosition
        //TODO: TESTING WITH OPENING ARM RIGHT FROM THE WALL. IF ARM GETS STUCK THEN UNCOMMENT THIS SECTION
       /*trajInitToOffWall = driveTrain.trajectorySequenceBuilder(initPose)
                   .lineToLinearHeading(offWallPose)
                   .build();*/

        //Move forward to Capstone pickup position and then to Carousal
        if (pickShippingElement) {
            for (int i = 0; i < 3; i++) {
                trajOffWallToBarCode[i] = driveTrain.trajectorySequenceBuilder(initPose) //offWallPose
                        .setVelConstraint(getVelocityConstraint(20, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(barcodePose[i])
                        .resetVelConstraint()
                        .build();
                trajASBarCodeToCarousal[i] = driveTrain.trajectorySequenceBuilder(barcodePose[i])
                        .lineToLinearHeading(carousalPose)
                        .build();
            }
        } else {
            trajOffWalltoCarousal = driveTrain.trajectorySequenceBuilder(initPose)
                    .lineToLinearHeading(offWallPose)
                    .addTemporalMarker(0,() -> {
                        majorArm.moveMajorArmWristToParkedPosition();
                    })
                    .lineToLinearHeading(carousalPose)
                    .build();
        }

        //Move from Carousal to Alliance Shipping Hub
        trajASCarousalToAlShipping = driveTrain.trajectorySequenceBuilder(carousalPose/*driveTrain.getPoseEstimate()*/)
                .addTemporalMarker(0, () -> {moveElevatorToTargetZoneLevel();})
                .lineToLinearHeading(carousalToAlliancePathPose) //Avoid Capstone
                .lineToLinearHeading(alShippingHubPose)
                .build();

        //Move from Alliance shipping o carousal
        trajASAlShippingToCarousalPickDuckToAllianceShipping = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                .addTemporalMarker(0, () -> {})
                .lineToLinearHeading(carousalToAlliancePathPose)
                .addTemporalMarker(0.5, () -> {
                    magazine.moveMagazineToCollect();
                    elevator.moveElevatorLevel0Position();
                    intake.startIntakeMotorInward();
                })
                .lineToLinearHeading(carousalPose)
                .lineToLinearHeading(pickDuckPose[0])
                .setVelConstraint(getVelocityConstraint(5, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(pickDuckPose[1])
                .lineToLinearHeading(pickDuckPose[0])
                .resetVelConstraint()
                .addTemporalMarker(()->{
                    magazine.moveMagazineToTransport();
                    intake.stopIntakeMotor();
                    elevator.moveElevatorLevel3Position();
                })
                .lineToLinearHeading(carousalToAlliancePathPose) //Avoid Capstone
                .lineToLinearHeading(alShippingHubPose)
                .addTemporalMarker(()->{
                    magazine.magazineServo.setPosition(0.22);
                })
                .build();
        if (parkingLocation == GameField.PARKING_LOCATION.STORAGE) {
            trajASAlShippingToStorageParking = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                    .lineToLinearHeading(carousalToAlliancePathPose) //Avoid Capstone
                    //.addTemporalMarker(1,()->{moveElevatorToLevel(1);})
                    .lineToLinearHeading(storageParkingPose)
                    .build();
        } else { //parkingLocation == GameField.PARKING_LOCATION.WAREHOUSE
            if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
                trajAlShippingToWHParking[0] = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                        .addTemporalMarker(0,()->{moveElevatorToLevel(1);})
                        .lineToLinearHeading(carousalToAlliancePathPose) //Avoid Capstone
                        .lineToLinearHeading(whAlongWallParkingPose[0])
                        .build();
                if (!GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                    if (loopsFromWarehouseToAlShippingHub == 0) {
                        trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                                .lineToLinearHeading(whAlongWallParkingPose[1])
                                .addTemporalMarker(0,() -> {
                                    moveElevatorToLevel(0);
                                })
                                .lineToLinearHeading(whAlongWallParkingPose[2])
                                .build();
                    } else {
                        trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                                .addTemporalMarker(0,() -> {
                                    moveElevatorToLevel(0);
                                    autonomousController.moveAutoMagazineToCollect();
                                    runIntakeToCollect();
                                })
                                .lineToLinearHeading(warehousePickElementPose[0])
                                .build();
                        buildWarehouseAllianceShippingLoopNew();
                    }
                } else { // GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                    trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                            .lineToLinearHeading(whAlongWallParkingPose[1])
                            .setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                            .lineToLinearHeading(whAlongWallShShippingParkingPose[0])
                            .lineToLinearHeading(whAlongWallShShippingParkingPose[1])
                            .resetVelConstraint()
                            .build();
                }
            } else { //(autonomousRoute == GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER)
                trajAlShippingToWHParking[0] = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                        .addTemporalMarker(0,()->{moveElevatorToLevel(1);})
                        .lineToLinearHeading(carousalToAlliancePathPose) //Avoid Capstone
                        .lineToLinearHeading(whThroughBarrierParkingPose[0])
                        .build();
                if (!GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                    trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whThroughBarrierParkingPose[0])
                            .lineToLinearHeading(whThroughBarrierParkingPose[1])
                            .build();

                } else { //GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                    trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whThroughBarrierParkingPose[0])
                            .lineToLinearHeading(whThroughBarrierParkingPose[1])
                            .setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                            .lineToLinearHeading(whThroughBarrierShShippingParkingPose)
                            .resetVelConstraint()
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
        //safeWait(100);

        //Move from wall to offWall position
        //TODO: TESTING WITH OPENING ARM RIGHT FROM THE WALL. IF ARM GETS STUCK THEN UNCOMMENT THIS SECTION
        //driveTrain.followTrajectorySequence(trajInitToOffWall);

        if (pickShippingElement) {
            //Move arm to Pickup Capstone level and open Grip
            moveMajorArmToPickupAndOpenClaw();

            //Move forward to Capstone Pickup Position
            driveTrain.followTrajectorySequence(trajOffWallToBarCode[targetZoneLevel]);

            //Collect Capstone and move arm to parking position
            moveMajorArmToParkingAfterClosingClaw();

            //Move to Carousal
            driveTrain.followTrajectorySequence(trajASBarCodeToCarousal[targetZoneLevel]);
        } else {
            driveTrain.followTrajectorySequence(trajOffWalltoCarousal);
        }

        //Rotate Carousal
        rotateCarousal();

        //Move To Shipping Unit
        driveTrain.followTrajectorySequence(trajASCarousalToAlShipping);

        //Drop pre-loaded box in correct level
        dropBoxToLevel();

        //Carousal to Shipping pose TO carousal pose
        driveTrain.followTrajectorySequence(trajASAlShippingToCarousalPickDuckToAllianceShipping);

        //Move to Parking
        if (parkingLocation == GameField.PARKING_LOCATION.STORAGE) {
            driveTrain.followTrajectorySequence(trajASAlShippingToStorageParking);
        } else {
            driveTrain.followTrajectorySequence(trajAlShippingToWHParking[0]);
            if (loopsFromWarehouseToAlShippingHub == 0) {
                while (gameTimer.time() < 22000) { //Wait till end of Autonomous mode
                    safeWait(100);
                }
                driveTrain.followTrajectorySequence(trajAlShippingToWHParking[1]);
            } else {
                driveTrain.followTrajectorySequence(trajAlShippingToWHParking[1]);
                for (int loop = 1; loop <= loopsFromWarehouseToAlShippingHub; loop++) {
                    //driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoop);
                    while (gameTimer.time() < 26000) {
                        if (senseIntakeCollectAndStop() == true) {
                            driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoopDrop[loop]);
                            safeWait(750);
                            driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoopPick[loop]);
                            break;
                        };
                    };
                }
            }
        }

        moveElevatorToLevel(0);
        autonomousController.stopAutoIntake();

        return;
    }

    public void buildAutoWarehouse(){
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            //BLUE_WAREHOUSE_STARTPOS =  new Pose2d(-61,7,Math.toRadians(180));
            initPose = GameField.BLUE_WAREHOUSE_STARTPOS;
            offWallPose = new Pose2d(-56,7,Math.toRadians(180));
            barcodePose[1-1] = new Pose2d(-45, 17, Math.toRadians(-170));
            barcodePose[2-1] = new Pose2d(-49, 9, Math.toRadians(180));
            barcodePose[3-1] = new Pose2d(-51, -1, Math.toRadians(165));

            alShippingHubPose = new Pose2d(-31, 2 , Math.toRadians(135));

            whAlongWallParkingPose[0] = new Pose2d(-72, 3, Math.toRadians(90)); //x -69
            whAlongWallParkingPose[1] = new Pose2d(-72, 36, Math.toRadians(90)); //x 54
            whAlongWallParkingPose[2] = new Pose2d(-72, 45, Math.toRadians(90)); //x 55

            warehousePickElementPose[0] = new Pose2d(-72, 47, Math.toRadians(90)); //y: 46;
            warehousePickElementPose[1] = new Pose2d(-73, 48, Math.toRadians(90)); //y: 47;
            warehousePickElementPose[2] = new Pose2d(-74, 52, Math.toRadians(90)); //y: 50;
            warehouseAllianceShippingPathPose[0] = new Pose2d(-72, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[1] = new Pose2d(-73, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[2] = new Pose2d(-74, 3, Math.toRadians(90));

            allianceShippingHubDropElementPose = new Pose2d(-32, -3 , Math.toRadians(135));

            whAlongWallShShippingParkingPose[0] = new Pose2d(-40, 36, Math.toRadians(45));
            whAlongWallShShippingParkingPose[1] = new Pose2d(-40, 68, Math.toRadians(0));

            whThroughBarrierParkingPose[0] = new Pose2d(-50, -10, Math.toRadians(60)); //x = -48
            whThroughBarrierParkingPose[1] = new Pose2d(-48, 60, Math.toRadians(90)); // x = -42
            whThroughBarrierShShippingParkingPose = new Pose2d(-45, 68, Math.toRadians(0)); // x = -40

        } else {
            //RED_WAREHOUSE_STARTPOS =  new Pose2d(61,7,Math.toRadians(0));
            initPose = GameField.RED_WAREHOUSE_STARTPOS;
            offWallPose = new Pose2d(56,7,Math.toRadians(0));
            barcodePose[1-1] = new Pose2d(51,8, Math.toRadians(-5)); //fixed 1/8/22
            barcodePose[2-1] = new Pose2d(49,8, Math.toRadians(-35)); //fixed 1/8/22
            barcodePose[3-1] = new Pose2d(49, 13, Math.toRadians(-45)); //fixed 1/8/22

            alShippingHubPose = new Pose2d(32, 0, Math.toRadians(45));

            whAlongWallParkingPose[0] = new Pose2d(70, 3, Math.toRadians(90)); //x -69
            whAlongWallParkingPose[1] = new Pose2d(70, 36, Math.toRadians(90)); //x 54
            whAlongWallParkingPose[2] = new Pose2d(70, 45, Math.toRadians(150)); //x55

            warehousePickElementPose[0] = new Pose2d(70, 47, Math.toRadians(90)); //y: 46;
            warehousePickElementPose[1] = new Pose2d(71, 48, Math.toRadians(90)); //y: 47;
            warehousePickElementPose[2] = new Pose2d(72, 52, Math.toRadians(90)); //y: 50
            warehouseAllianceShippingPathPose[0] = new Pose2d(70, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[1] = new Pose2d(71, 3, Math.toRadians(90));
            warehouseAllianceShippingPathPose[2] = new Pose2d(72, 3, Math.toRadians(90));

            allianceShippingHubDropElementPose = new Pose2d(32, -3, Math.toRadians(45));

            whAlongWallShShippingParkingPose[0] = new Pose2d(40, 36, Math.toRadians(135));
            whAlongWallShShippingParkingPose[1] = new Pose2d(40, 68, Math.toRadians(180));

            whThroughBarrierParkingPose[0] = new Pose2d(50, -10, Math.toRadians(120));
            whThroughBarrierParkingPose[1] = new Pose2d(48, 60, Math.toRadians(90));
            whThroughBarrierShShippingParkingPose = new Pose2d(45, 68, Math.toRadians(180));
        }

        //Move from init to offWallPosition
        //TODO: TESTING WITH OPENING ARM RIGHT FROM THE WALL. IF ARM GETS STUCK THEN UNCOMMENT THIS SECTION
       /*trajInitToOffWall = driveTrain.trajectorySequenceBuilder(initPose)
               .lineToLinearHeading(offWallPose)
               .build();*/

        if (pickShippingElement) {
            //Move forward to Capstone pickup position and then to Carousal
            for (int i = 0; i < 3; i++) {
                trajOffWallToBarCode[i] = driveTrain.trajectorySequenceBuilder(initPose/*offWallPose*/)
                        .setVelConstraint(getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(barcodePose[i])
                        .resetVelConstraint()
                        .build();
                //Move from Barcode to Alliance Shipping Hub
                trajAWBarCodeToAlShipping[i] = driveTrain.trajectorySequenceBuilder(barcodePose[i])
                        .addTemporalMarker(0,() -> {
                            moveElevatorToTargetZoneLevel();
                        })
                        .lineToLinearHeading(alShippingHubPose)
                        .build();
            }
        } else {
            trajOffWalltoAlShipping = driveTrain.trajectorySequenceBuilder(initPose)
                    .addTemporalMarker(0,() -> {
                        moveElevatorToTargetZoneLevel();
                    })
                    .lineToLinearHeading(alShippingHubPose)
                    .build();
        }

        //Move to Alliance Shipping Hub to Warehouse Parking
        if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
            trajAlShippingToWHParking[0] = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                    //.setVelConstraint(getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .addTemporalMarker(0,()->{
                        moveElevatorToLevel(0);
                    })
                    .lineToLinearHeading(whAlongWallParkingPose[0])
                    //.resetVelConstraint()
                    .build();
            if (!GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                if (loopsFromWarehouseToAlShippingHub == 0) {
                    trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                            .lineToLinearHeading(whAlongWallParkingPose[1])
                            .lineToLinearHeading(whAlongWallParkingPose[2])
                            .build();
                } else {
                    trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                            .setVelConstraint(getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                            .addTemporalMarker(0,() -> {
                                runIntakeToCollect();
                            })
                            .lineToLinearHeading(warehousePickElementPose[0])
                            .resetVelConstraint()
                            .build();
                    //buildWarehouseAllianceShippingLoop();
                    buildWarehouseAllianceShippingLoopNew();
                }
            } else { // GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whAlongWallParkingPose[0])
                        .lineToLinearHeading(whAlongWallParkingPose[1])
                        //.setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(whAlongWallShShippingParkingPose[0])
                        .lineToLinearHeading(whAlongWallShShippingParkingPose[1])
                        //.resetVelConstraint()
                        .build();
            }
        } else { //(autonomousRoute == GameField.AUTONOMOUS_ROUTE.THROUGH_BARRIER)
            trajAlShippingToWHParking[0] = driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                    .addTemporalMarker(0,()->{moveElevatorToLevel(1);})
                    .lineToLinearHeading(whThroughBarrierParkingPose[0])
                    .build();
            if (!GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB) {
                trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whThroughBarrierParkingPose[0])
                        .lineToLinearHeading(whThroughBarrierParkingPose[1])
                        .build();

            } else { // ! GameField.END_PARKING_FACING_SHARED_SHIPPING_HUB
                trajAlShippingToWHParking[1] = driveTrain.trajectorySequenceBuilder(whThroughBarrierParkingPose[0])
                        .lineToLinearHeading(whThroughBarrierParkingPose[1])
                        .setVelConstraint(getVelocityConstraint(30, DriveConstants.MAX_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(whThroughBarrierShShippingParkingPose)
                        .build();
            }
        }
    }


    /**
     * Path and actions for autonomous mode starting from Inner start position
     */
    public void runAutoWarehouse(){
        //Logic for waiting
        //safeWait(100);

        //Move from wall to offWall position
        //driveTrain.followTrajectorySequence(trajInitToOffWall);

        if (pickShippingElement) {
            //Move arm to Pickup Capstone level and open Grip
            moveMajorArmToPickupAndOpenClaw();

            //Move forward to Capstone Pickup Position
            driveTrain.followTrajectorySequence(trajOffWallToBarCode[targetZoneLevel]);

            //Collect Capstone and move arm to parking position
            moveMajorArmToParkingAfterClosingClaw();

            //Move to Alliance Shipping Hub
            driveTrain.followTrajectorySequence(trajAWBarCodeToAlShipping[targetZoneLevel]);
        } else {
            driveTrain.followTrajectorySequence(trajOffWalltoAlShipping);
        }

        //Drop pre-loaded box in correct level
        dropBoxToLevel();

        //Move to Parking
        driveTrain.followTrajectorySequence(trajAlShippingToWHParking[0]);
        driveTrain.followTrajectorySequence(trajAlShippingToWHParking[1]);
        if (loopsFromWarehouseToAlShippingHub == 0) {

        } else {
            for (int loop = 0; loop < loopsFromWarehouseToAlShippingHub; loop++) {

                while (gameTimer.time() < 26000) {
                    if (senseIntakeCollectAndStop() == true) {
                        driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoopDrop[loop]);
                        loopWait(800);
                        if(!(loop == loopsFromWarehouseToAlShippingHub - 1)){
                            driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoopPick[loop]);
                        } else {
                            if (!whLoopParkThroughBarrier) {
                                driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingLoopPick[loop]);
                            } else {
                                driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingToParkBarrier);
                            }
                        }
                        break;
                    };
                };
                if (whLoopParkThroughBarrier) {
                    driveTrain.followTrajectorySequence(trajWarehouseAllianceShippingToParkBarrier);
                }
            }
        }

        moveElevatorToLevel(0);
        autonomousController.moveAutoMagazineToCollect();
        autonomousController.stopAutoIntake();
        return;
    }





    public void buildWarehouseAllianceShippingLoopNew() {
        for (int loop = 0; loop < loopsFromWarehouseToAlShippingHub; loop++) {
            trajWarehouseAllianceShippingLoopDrop[loop] = driveTrain.trajectorySequenceBuilder(warehousePickElementPose[loop])
                    //.setVelConstraint(getVelocityConstraint(80, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                    .lineToLinearHeading(warehouseAllianceShippingPathPose[loop])
                    .addTemporalMarker(0.5, () -> {
                        //autonomousController.moveAutoMagazineToTransport();
                        magazine.moveMagazineToTransport();
                        intake.startIntakeMotorOutward();
                        //moveElevatorToLevel(3);
                        elevator.moveElevatorLevel3Position();
                    })
                    /*.addTemporalMarker(1, () -> {
                        intake.stopIntakeMotor();
                    })
                     */
                    .lineToLinearHeading(allianceShippingHubDropElementPose)
                    .addTemporalMarker(() -> {
                        //autonomousController.moveAutoMagazineToDrop();
                        magazine.moveMagazineToDrop();
                    })
                    .build();
            trajWarehouseAllianceShippingLoopPick[loop] = driveTrain.trajectorySequenceBuilder(allianceShippingHubDropElementPose)
                    .lineToLinearHeading(warehouseAllianceShippingPathPose[loop])
                    .addTemporalMarker(0.5, () -> {
                        //autonomousController.moveAutoMagazineToCollect();
                        magazine.moveMagazineToCollect();
                        //moveElevatorToLevel(0);
                        elevator.moveElevatorLevel0Position();
                        //runIntakeToCollect();
                        intake.startIntakeMotorInward();
                    })
                    .lineToLinearHeading(warehousePickElementPose[loop])
                    //.resetVelConstraint()
                    .build();

        }

        trajWarehouseAllianceShippingToParkBarrier= driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                .addTemporalMarker(0,()->{moveElevatorToLevel(1);})
                .lineToLinearHeading(whThroughBarrierParkingPose[0])
                .lineToLinearHeading(whThroughBarrierParkingPose[1])
                .build();
    }


    public void rotateCarousal() {
        if (GameField.playingAlliance == GameField.PLAYING_ALLIANCE.BLUE_ALLIANCE) {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.CLOCKWISE;
        } else {
            autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.ANTICLOCKWISE;
        }
        autonomousController.runAutoControl();
        safeWait(2300);
        autonomousController.autoSpinnerState = AutonomousController.AUTO_SPINNER_STATE.STOPPED;
        autonomousController.runAutoControl();
    }

    //Drops pre-loaded box at the correct level determined by capstone position
    public void dropBoxToLevel(){
        autonomousController.moveAutoMagazineToDrop();
        safeWait(1000);
        autonomousController.moveAutoMagazineToTransport();
        safeWait(100);
    }


    //Runs the Intake after moving the Elevator to Level0 and magazine to collect
    public void runIntakeToCollect(){
        autonomousController.moveAutoMagazineToCollect();
        autonomousController.moveAutoElevatorLevel0();
        autonomousController.startAutoIntake();
    }

    public boolean senseIntakeCollectAndStop(){
        if (magazine.getMagazineColorSensorState() == Magazine.MAGAZINE_COLOR_SENSOR_STATE.LOADED) {
            return true;
        }
        return false;
    }


    public void moveElevatorToLevel(int level){
        switch (level) {
            case 0 : autonomousController.moveAutoElevatorLevel0(); break;
            case 1 : autonomousController.moveAutoElevatorLevel1(); break;
            case 2 : autonomousController.moveAutoElevatorLevel2(); break;
            case 3 : autonomousController.moveAutoElevatorLevel3(); break;
        }
    }

    public void moveElevatorToTargetZoneLevel(){
        switch (targetZone) {
            case LEVEL1: autonomousController.moveAutoElevatorLevel1(); break;
            case LEVEL2: autonomousController.moveAutoElevatorLevel2(); break;
            case LEVEL3: autonomousController.moveAutoElevatorLevel3(); break;
        }
    }


    /**
     * Hybrid Commands For Autonomous OpMode
     */

    public void moveMajorArmToParkingAfterClosingClaw(){
        //safeWait(1000);
        autonomousController.closeAutoMajorClaw();
        safeWait(200);
        autonomousController.moveAutoMajorArmCapstoneDrop();
        safeWait(200);
        majorArm.moveMajorArmWristToParkedPosition();
        safeWait(300);
        autonomousController.moveAutoMajorArmPark();
        safeWait(250);
    }


    public void moveMajorArmToPickupAndOpenClaw(){
        majorArm.moveMajorArmWristToPickupPosition();
        //safeWait(100);
        autonomousController.moveAutoMajorArmCapstonePickup();
        //safeWait(750);
        autonomousController.openAutoMajorClaw();
        safeWait(2000); //1000
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

    public void loopWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            //autonomousController.runAutoControl();
            driveTrain.update();
        }
    }

    public void selectGamePlan(){
        telemetry.setAutoClear(true);
        telemetry.addData("Compile time : ", "22:00 :: 1/27/2022");

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

        while (!isStopRequested()) {
            telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
            telemetry.addData("StartPosition : ", GameField.startPosition);
            telemetry.addData("Pick Game Shipping Element :","");
            telemetry.addData("  YES (Y)","");
            telemetry.addData("  NO (A)","");
            if (gamepadController.gp1GetButtonYPress()) {
                pickShippingElement = true;
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                break;
            }
            if (gamepadController.gp1GetButtonAPress()) {
                pickShippingElement = false;
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                break;
            }
            telemetry.update();
        }


        if (GameField.startPosition == GameField.START_POSITION.WAREHOUSE) {
            parkingLocation = GameField.PARKING_LOCATION.WAREHOUSE;
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Parking Location: ",parkingLocation);
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
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

            if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
                while (!isStopRequested()) {
                    telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                    telemetry.addData("StartPosition : ", GameField.startPosition);
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    telemetry.addData("Parking Location: ", parkingLocation);
                    telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                    telemetry.addData("No. of Loop from Warehouse to AlShipping Hub :", "");
                    telemetry.addData("  0 (A)", "");
                    telemetry.addData("  1 (X)", "");
                    if (!pickShippingElement){
                        telemetry.addData("  2 (Y)", "");
                    }
                    //telemetry.addData("  3 (B)", "");
                    if (gamepadController.gp1GetButtonAPress()) {
                        loopsFromWarehouseToAlShippingHub = 0;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    if (gamepadController.gp1GetButtonXPress()) {
                        loopsFromWarehouseToAlShippingHub = 1;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    if (!pickShippingElement) {
                        if (gamepadController.gp1GetButtonYPress()) {
                            loopsFromWarehouseToAlShippingHub = 2;
                            telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                            break;
                        }
                    }
                    /*if (gamepadController.gp1GetButtonBPress()) {
                       loopsFromWarehouseToAlShippingHub = 3;
                       telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                       break;
                    }*/
                    telemetry.update();
                }
            }
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Autonomous Route : ", autonomousRoute);
                telemetry.addData("Parking Location: ", parkingLocation);
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                telemetry.addData("No. of Loop from Warehouse to AlShipping Hub :", "");
                telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                telemetry.addData("Park along wall or through barrier :","");
                telemetry.addData("  Loop PARK ALONG WALL (X)","");
                telemetry.addData("  Loop PARK THROUGH BARRIER (B)","");
                if (gamepadController.gp1GetButtonXPress()) {
                    whLoopParkThroughBarrier = false;
                    telemetry.addData("Loop PARK THROUGH BARRIER : ", whLoopParkThroughBarrier);
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    whLoopParkThroughBarrier = true;
                    telemetry.addData("Loop PARK THROUGH BARRIER : ", whLoopParkThroughBarrier);
                    break;
                }
                telemetry.update();
            }
        }

        if (GameField.startPosition == GameField.START_POSITION.STORAGE) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
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

            if (autonomousRoute == GameField.AUTONOMOUS_ROUTE.ALONG_WALL) {
                while (!isStopRequested()) {
                    telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                    telemetry.addData("StartPosition : ", GameField.startPosition);
                    telemetry.addData("Autonomous Route : ", autonomousRoute);
                    telemetry.addData("Parking Location: ", parkingLocation);
                    telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                    telemetry.addData("No. of Loop from Warehouse to AlShipping Hub :", "");
                    telemetry.addData("  0 (A)", "");
                    telemetry.addData("  1 (X)", "");
                    //telemetry.addData("  2 (Y)", "");
                    //telemetry.addData("  3 (B)", "");
                    if (gamepadController.gp1GetButtonAPress()) {
                        loopsFromWarehouseToAlShippingHub = 0;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    if (gamepadController.gp1GetButtonXPress()) {
                        loopsFromWarehouseToAlShippingHub = 1;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    if (gamepadController.gp1GetButtonYPress()) {
                        loopsFromWarehouseToAlShippingHub = 2;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    if (gamepadController.gp1GetButtonBPress()) {
                        loopsFromWarehouseToAlShippingHub = 3;
                        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
                        break;
                    }
                    telemetry.update();
                }
            }
        }



        if (parkingLocation == GameField.PARKING_LOCATION.WAREHOUSE &&
                loopsFromWarehouseToAlShippingHub == 0) {
            while (!isStopRequested()) {
                telemetry.addData("Playing Alliance Selected : ", GameField.playingAlliance);
                telemetry.addData("StartPosition : ", GameField.startPosition);
                telemetry.addData("Autonomous Route : ", autonomousRoute);
                telemetry.addData("Parking Location: ", parkingLocation);
                telemetry.addData("Pick Shipping Element : ", pickShippingElement);
                telemetry.addData("No. of Loops from WH to AlShipping Hub : ", loopsFromWarehouseToAlShippingHub);
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
        telemetry.addData("Pick Shipping Element : ", pickShippingElement);
        telemetry.addData("No. of Loops from WH to AlShippingHub : ", loopsFromWarehouseToAlShippingHub);
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
        telemetry.addData("Pick Shipping Element : ", pickShippingElement);

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

        telemetry.addData("Blinkin Pattern : ", blinkinDisplay.currentPattern);
        telemetry.addData("Game Timer : ", gameTimer.time());

        telemetry.update();

    }
}



