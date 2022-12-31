package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example
 */
@Autonomous(name = "Hazmat Auto", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutoOpMode extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum AUTO_OPTION{
        FULL_AUTO,
        DROP_PRELOAD_AND_PARK,
        ONLY_PARK
    }
    public static AUTO_OPTION autoOption;

    public enum DROP_CONE_POSITION {
        MEDIUM,
        HIGH
    }
    public static DROP_CONE_POSITION dropConePosition = DROP_CONE_POSITION.HIGH;

    public int pickAndDropConeCount = 1;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public OuttakeArm outtakeArm;
    public OuttakeSlides outtakeSlides;
    public Lights lights;
    public Vision vision;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
        GameField.opModeRunning = GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;
        /* Set Initial State of any subsystem when OpMode is to be started*/
        initSubsystems();

        //Key Pay inputs to selecting Starting Position of robot
        selectStartingPosition();

        // Initiate Camera on Init.
        vision.activateVuforiaTensorFlow();

        //Build Autonomous trajectory to be used based on starting position selected
        buildAuto();
        driveTrain.getLocalizer().setPoseEstimate(initPose);

        lights.setPatternYellow();

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addLine("Start Hazmat Autonomous Mode");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("AutoOption Selected", autoOption);
            telemetry.addData("Vision identified Parking Location", vision.visionIdentifiedTarget);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            //Turn Lights Green
            lights.setPatternGreen();

            //Stop Vision process
            vision.deactivateVuforiaTensorFlow();

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //Turn Auto Demo light patten on
            lights.setPatternDemo();

            //run Autonomous trajectory
            runAutoAndParking();
        }

        //Trajectory is completed, display Parking complete
        parkingComplete();

        lights.setPatternBlack();

        //Write last position to static class to be used as initial position in TeleOp
        GameField.currentPose = driveTrain.getPoseEstimate();
        GameField.poseSetInAutonomous = true;
    }

    //Initialize any other TrajectorySequences as desired
    TrajectorySequence trajectoryAuto, trajectoryParking ;

    //Initialize Robot Positions (Pose2d's)
    Pose2d initPose; // Starting Pose
    Pose2d midWayPose;
    Pose2d pickAndDropHighPose;
    Pose2d pickAndDropMediumPose;
    Pose2d pickAndDropPose;
    Pose2d parkPose;

    OuttakeSlides.TURRET_STATE pickAndDropTurretStateHigh;
    OuttakeSlides.TURRET_STATE pickAndDropTurretStateMedium;
    OuttakeSlides.TURRET_STATE pickAndDropTurretState;

    int coneCount = 0;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-64, 36, Math.toRadians(0)); //Starting pose
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(-12, 39, Math.toRadians(90));
                pickAndDropMediumPose = new Pose2d(-12, 33, Math.toRadians(90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-64, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(-12, -39, Math.toRadians(-90));
                pickAndDropMediumPose = new Pose2d(-12, -33, Math.toRadians(-90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                break;

            case RED_LEFT:
                initPose = new Pose2d(64, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone, 180
                pickAndDropHighPose = new Pose2d(12, -39, Math.toRadians(270));
                pickAndDropMediumPose = new Pose2d(12, -33, Math.toRadians(270));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                break;

            case RED_RIGHT:
                initPose = new Pose2d(64, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(12, 39, Math.toRadians(90));
                pickAndDropMediumPose = new Pose2d(12, 33, Math.toRadians(90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                break;
        }
        switch (dropConePosition) {
            case MEDIUM:
                pickAndDropPose = pickAndDropMediumPose;
                pickAndDropTurretState = pickAndDropTurretStateMedium;
                break;
            case HIGH:
                pickAndDropPose = pickAndDropHighPose;
                pickAndDropTurretState = pickAndDropTurretStateHigh;
                break;
        }

        //Move forward to midWayPose, rotate turret to 0 and reset turret
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
              .addTemporalMarker(0.1 ,() -> {
                    outtakeSlides.moveTurret(pickAndDropTurretState);
              })

                .setVelConstraint(getVelocityConstraint(30, 15, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(midWayPose)
                .lineToLinearHeading(pickAndDropPose)
                .resetVelConstraint()
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 12, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -12, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -12, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, 12, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, 60, Math.toRadians(180)); break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(midWayPose)
                .lineToLinearHeading(parkPose)
                .build();
    }

    //Run Auto trajectory and parking trajectory
    public void runAutoAndParking(){
        telemetry.setAutoClear(false);
        telemetry.addLine("Hazmat Autonomous Mode");
        telemetry.addLine("----------------------------------");
        telemetry.addData("Selected Starting Position",startPosition);
        telemetry.addData("Selected Auto Option", autoOption);
        if (autoOption != AUTO_OPTION.ONLY_PARK) {
            telemetry.addData("Selected dropConePosition", dropConePosition);
        }
        if (autoOption == AUTO_OPTION.FULL_AUTO) {
            telemetry.addData("Selected number of pickAndDropCone", pickAndDropConeCount);
        }
        telemetry.update();
        //Starting AadiPose - Arm max retracted, shoulder at Intake, turret facing +45degrees, camera to the front

        //Move forward to midWayPose, rotate turret to 0 and reset turret.
        driveTrain.followTrajectorySequence(trajectoryAuto);
        //turret.resetTurretMode();

        //turn turret and pick, then drop cone
        if (autoOption == AUTO_OPTION.ONLY_PARK) {
            dropCone(); //Drop preloaded Cone
        } else { //TODO: Write state machine for intake and outtake looping
            /*
            for (coneCount = 1; coneCount <= pickAndDropConeCount; coneCount++) {
                pickCone(coneCount);
                dropCone();
            }
             */
        }

        //turret.rotateAutoTurnToAngle(0);
        safeWait(500);
        driveTrain.followTrajectorySequence(trajectoryParking);
        //gamepadController.moveToNeutralPickup();
        //hand.moveWristUpMax();
        //hand.closeGrip();
        safeWait(1000);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(int coneCount) {
        //Open Grip and rotate to pickConeAadiPose
        //hand.openGrip();
        //turret.rotateAutoTurnToAngle(pickConeAadiPose.getTurretAngle());
        safeWait(500);

        //gamepadController.moveToNeutralLow();
        //safeWait(2000);

        //Move Arm to pickCone Pose
        //gamepadController.moveToAadiVector(pickConeAadiPose.getAadiVector(), pickConeAadiPose.getWristState());
        //gamepadController.runArmShoulderWristToLevel();
        //hand.moveWristLevel(shoulder.shoulderCurrentPosition);
        safeWait(1000);

        //Close grip
        //hand.closeGrip();
        safeWait(1500);

        //Raise shoulder to clear from stack and wrist up
        //gamepadController.raiseShoulderToClearStack();
        safeWait(500);
        //hand.moveWristUp(shoulder.shoulderCurrentPosition);
        safeWait(500);

        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(){
        //Raise arm to Neutral High and wrist up
        //gamepadController.moveToNeutralHigh();
        //safeWait(500);
        //hand.moveWristUp(shoulder.shoulderCurrentPosition);

        //Rotate turret to dropConePose
        //turret.rotateAutoTurnToAngle(dropConeAadiPose.getTurretAngle());
        //telemetry.addData("dropConeAadiPose.getTurretAngle()",dropConeAadiPose.getTurretAngle() );
        //telemetry.addData("turret.turretCurrentPosition", turret.turretCurrentPosition);
        telemetry.update();
        safeWait(1000);

        //Move Arm to dropCone Post, wrist level
        //gamepadController.moveToAadiVector(dropConeAadiPose.getAadiVector(), dropConeAadiPose.getWristState());
        //gamepadController.runArmShoulderWristToLevel();
        safeWait(1000);

        //Open grip to drop Cone
        //hand.openGrip();
        safeWait(1000);

        //Move arm to neutral high, wrist level
        //gamepadController.moveToNeutralHigh();
        safeWait(500);

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    public void parkingComplete(){
        telemetry.addData("Parked in Location", vision.visionIdentifiedTarget);
        telemetry.update();
    }

    //Method to select starting position using X, Y, A, B buttons on gamepad
    public void selectStartingPosition() {
        telemetry.setAutoClear(true);
        telemetry.clearAll();
        //******select start pose*****
        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Select Starting Position using XYAB Keys on gamepad 1:","");
            telemetry.addData("    Blue Left   ", "(X / Square)");
            telemetry.addData("    Blue Right ", "(Y / Triangle)");
            telemetry.addData("    Red Left    ", "(A / Cross)");
            telemetry.addData("    Red Right  ", "(B / Circle)");
            if(gamepadController.gp1GetButtonXPress()){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepadController.gp1GetButtonAPress()){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
        }

        while(!isStopRequested()){
            telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position",startPosition);
            telemetry.addLine("Select Auto Options");
            telemetry.addData("    Full Autonomous                ","X / Square");
            telemetry.addData("    Drop Preloaded and Park   ","Y / Triangle");
            telemetry.addData("    Only Park                      ","B / Circle");
            if(gamepadController.gp1GetButtonXPress()){
                autoOption = AUTO_OPTION.FULL_AUTO;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                autoOption = AUTO_OPTION.DROP_PRELOAD_AND_PARK;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                autoOption = AUTO_OPTION.ONLY_PARK;
                break;
            }
            telemetry.update();
        }

        if (autoOption != AUTO_OPTION.ONLY_PARK) {
            while (!isStopRequested()) {
                telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team", "TEAM NUMBER");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addLine("Select dropCone Postition");
                telemetry.addData("    Medium          ", "Y / Triangle");
                telemetry.addData("    High            ", "B / Circle");
                if (gamepadController.gp1GetButtonYPress()) {
                    dropConePosition = DROP_CONE_POSITION.MEDIUM;
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    dropConePosition = DROP_CONE_POSITION.HIGH;
                    break;
                }
                telemetry.update();
            }
        }

        if (autoOption == AUTO_OPTION.FULL_AUTO) {
            while (!isStopRequested()) {
                telemetry.addData("Initializing FTC Wires (ftcwires.org) Autonomous Mode adopted for Team", "TEAM NUMBER");
                telemetry.addData("---------------------------------------", "");
                telemetry.addData("Selected Starting Position", startPosition);
                telemetry.addData("Selected Auto Option", autoOption);
                telemetry.addData("Selected dropCone Position", dropConePosition);

                telemetry.addLine("Select number of cones to pick and drop");
                telemetry.addLine("(Use dpad up to increase and dpad down to reduce)");
                telemetry.addLine("(Once final, press X / Square to finalize)");

                telemetry.addData("    pickAndDropConeCount", pickAndDropConeCount);

                if (gamepadController.gp1GetDpad_upPress()) {
                    pickAndDropConeCount += 1;
                    if (pickAndDropConeCount > 5) {
                        pickAndDropConeCount = 5;
                    }
                }
                if (gamepadController.gp1GetDpad_downPress()) {
                    pickAndDropConeCount -= 1;
                    if (pickAndDropConeCount < 1) {
                        pickAndDropConeCount = 1;
                    }
                }

                if (gamepadController.gp1GetButtonXPress()) {
                    telemetry.addData("Selected number of pickAndDropCones", pickAndDropConeCount);
                    break;
                }

                telemetry.update();
            }
        }

        telemetry.clearAll();
    }

    public void initSubsystems(){

        telemetry.setAutoClear(false);

        //Init Pressed
        telemetry.addLine("Robot Init Pressed");
        telemetry.addLine("==================");
        telemetry.update();

        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);
        telemetry.addLine("DriveTrain Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap);
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        intakeSlides = new IntakeSlides(hardwareMap);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap);
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        lights = new Lights(hardwareMap);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intakeArm, intakeSlides, outtakeArm, outtakeSlides, lights);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        for (LynxModule module : hardwareMap.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, All systems Go! Let countdown begin. Waiting for Start");
        telemetry.update();
    }

    public void safeWait(double time) {
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
            driveTrain.update();
        }
    }

}