package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive.getVelocityConstraint;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AadiGeometry.AadiPose;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example
 */
@Autonomous(name = "Hazmat 1+3 Lakeshore", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutonomousLakeShoreOnePlus3 extends LinearOpMode{

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
        FRONT_HIGH,
        BACK_MEDIUM,
        BACK_HIGH
    }
    public static DROP_CONE_POSITION dropConePosition = DROP_CONE_POSITION.FRONT_HIGH;

    public int pickAndDropConeCount = 1;

    public Vision vision;
    public DriveTrain driveTrain;
    public Hand hand;
    public Arm arm;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;
    public GamepadController gamepadController;

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
    Pose2d parkPose;

    //Initialize Arm/Shoulder/Turret Positions (AadiPoses)
    //AadiPose initAadiPose;
    AadiPose[] pickConeAadiPose = new AadiPose[6];
    AadiPose dropConeAadiPose;
    AadiPose dropConeFrontHigh, dropConeBackMedium, dropConeBackHigh;
    //AadiPose endAadiPose;

    int coneCount = 0;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-64, 36, Math.toRadians(0)); //Starting pose
                //initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(-11, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone

                //midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConeFrontHigh = new AadiPose(1248,937, Hand.WRIST_STATE.WRIST_UP, 325);
                dropConeBackMedium = new AadiPose(450,730, Hand.WRIST_STATE.WRIST_UP, -1700);;
                dropConeBackHigh = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1850);
                pickConeAadiPose[1] = new AadiPose(2073,330, Hand.WRIST_STATE.WRIST_LEVEL, -660); // arm 1st : 2089, shoulder 1st : 295, arm 2nd: 832, shoulder 2nd: 175
                pickConeAadiPose[2] = new AadiPose(2261,315, Hand.WRIST_STATE.WRIST_LEVEL, -660); //shoulder first angle 275,  shoulder second: 266, arm second: 957
                pickConeAadiPose[3] = new AadiPose(2180,290, Hand.WRIST_STATE.WRIST_LEVEL, -660); //shoulder first angle 225, arm second: 983, shoulder second: 177
                pickConeAadiPose[4] = new AadiPose(2327,229, Hand.WRIST_STATE.WRIST_LEVEL, -660); // shoulder 210, shoulder second: 132, arm second: 1108
                pickConeAadiPose[5] = new AadiPose(2404,165, Hand.WRIST_STATE.WRIST_LEVEL, -660);
                //endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_LEVEL, 0);
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-64, -36, Math.toRadians(0));//Starting pose
                //initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(-11, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConeFrontHigh = new AadiPose(1420,930, Hand.WRIST_STATE.WRIST_UP, -279);
                dropConeBackMedium = new AadiPose(450,730, Hand.WRIST_STATE.WRIST_UP, 1700);;
                dropConeBackHigh = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1850);
                pickConeAadiPose[1] = new AadiPose(2140,347, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[2] = new AadiPose(2206,322, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[3] = new AadiPose(2259,297, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[4] = new AadiPose(2219,206, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[5] = new AadiPose(2379,160, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                //endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_LEVEL, 0);
                break;

            case RED_LEFT:
                initPose = new Pose2d(64, -36, Math.toRadians(180));//Starting pose
                //initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(13, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone, 180
                dropConeFrontHigh = new AadiPose(1248,946, Hand.WRIST_STATE.WRIST_UP, 353); //781 2nd pos shoulder, 1874 arm 2nd pos
                dropConeBackMedium = new AadiPose(450,730, Hand.WRIST_STATE.WRIST_UP, -1700);;
                dropConeBackHigh = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1850);
                pickConeAadiPose[1] = new AadiPose(2075,333, Hand.WRIST_STATE.WRIST_LEVEL, -650); // arm 1st : 2089, shoulder 1st : 295, arm 2nd: 832, shoulder 2nd: 175
                pickConeAadiPose[2] = new AadiPose(2263,317, Hand.WRIST_STATE.WRIST_LEVEL, -650); //shoulder first angle 275,  shoulder second: 266, arm second: 957
                pickConeAadiPose[3] = new AadiPose(2182,293, Hand.WRIST_STATE.WRIST_LEVEL, -650); //shoulder first angle 225, arm second: 983, shoulder second: 177
                pickConeAadiPose[4] = new AadiPose(2329,232, Hand.WRIST_STATE.WRIST_LEVEL, -650); // shoulder 210, shoulder second: 132, arm second: 1108
                pickConeAadiPose[5] = new AadiPose(2406,167, Hand.WRIST_STATE.WRIST_LEVEL, -650); // shoulder 115, shoulder second:44  , arm second: 1117
                break;

            case RED_RIGHT:
                initPose = new Pose2d(64, 36, Math.toRadians(180)); //Starting pose
                //initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(11, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                dropConeFrontHigh = new AadiPose(1450,930, Hand.WRIST_STATE.WRIST_UP, -290);
                dropConeBackMedium = new AadiPose(450,730, Hand.WRIST_STATE.WRIST_UP, 1700);;
                dropConeBackHigh = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1850);
                pickConeAadiPose[1] = new AadiPose(2140,347, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[2] = new AadiPose(2206,322, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[3] = new AadiPose(2259,297, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[4] = new AadiPose(2219,206, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                pickConeAadiPose[5] = new AadiPose(2379,160, Hand.WRIST_STATE.WRIST_LEVEL, 660);
                //endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_LEVEL, 0);
                break;
        }

        //Move forward to midWayPose, rotate turret to 0 and reset turret
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .addTemporalMarker(0.1 ,() -> {
                    turret.rotateAutoInitTurn();
                })
                .setVelConstraint(getVelocityConstraint(40, 15, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(midWayPose)
                .resetVelConstraint()
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-36, 61, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-36, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-36, 11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-36, -11, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-36, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-36, -61, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(36, -61, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(36, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(36, -11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(36, 11, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(36, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(36, 61, Math.toRadians(180)); break; // Location 3
                }
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(midWayPose)
                .back(21)
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

        switch (dropConePosition) {
            case FRONT_HIGH: dropConeAadiPose = dropConeFrontHigh; break;
            case BACK_MEDIUM: dropConeAadiPose = dropConeBackMedium; break;
            case BACK_HIGH: dropConeAadiPose = dropConeBackHigh; break;
        }

        //Starting AadiPose - Arm max retracted, shoulder at Intake, turret facing +45degrees, camera to the front

        //Move forward to midWayPose, rotate turret to 0 and reset turret.
        driveTrain.followTrajectorySequence(trajectoryAuto);
        turret.resetTurretMode();

        //turn turret and pick, then drop cone
        if (autoOption != AUTO_OPTION.ONLY_PARK) {
            dropCone(dropConeAadiPose); //Drop preloaded Cone

            if (autoOption == AUTO_OPTION.FULL_AUTO) {
                for (coneCount = 1; coneCount <= pickAndDropConeCount; coneCount++) {
                    pickCone(pickConeAadiPose[coneCount]);
                    dropCone(dropConeAadiPose);
                }
            }
        }
        turret.rotateAutoTurnToAngle(0);
        safeWait(400);
        driveTrain.followTrajectorySequence(trajectoryParking);
        gamepadController.moveToNeutralPickup();
        hand.moveWristUpMax();
        hand.closeGrip();
        safeWait(750);
    }

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(AadiPose pickConeAadiPose) {
        //Open Grip and rotate to pickConeAadiPose
        hand.openGrip();
        turret.rotateAutoTurnToAngle(pickConeAadiPose.getTurretAngle());
        safeWait(400);

        //gamepadController.moveToNeutralLow();
        //safeWait(2000);

        //Move Arm to pickCone Pose
        gamepadController.moveToAadiVector(pickConeAadiPose.getAadiVector(), pickConeAadiPose.getWristState());
        gamepadController.runArmShoulderWristToLevel();
        hand.moveWristLevel(shoulder.shoulderCurrentPosition);
        safeWait(750);

        //Close grip
        hand.closeGrip();
        safeWait(1000);

        //Raise shoulder to clear from stack and wrist up
        gamepadController.raiseShoulderToClearStack();
        safeWait(400);
        hand.moveWristUp(shoulder.shoulderCurrentPosition);
        safeWait(400);

        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
}

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(AadiPose dropConeAadiPose){
        //Raise arm to Neutral High and wrist up
        gamepadController.moveToNeutralHigh();
        //safeWait(500);
        hand.moveWristUp(shoulder.shoulderCurrentPosition);

        //Rotate turret to dropConePose
        turret.rotateAutoTurnToAngle(dropConeAadiPose.getTurretAngle());
        telemetry.addData("dropConeAadiPose.getTurretAngle()",dropConeAadiPose.getTurretAngle() );
        telemetry.addData("turret.turretCurrentPosition", turret.turretCurrentPosition);
        telemetry.update();
        safeWait(750);

        //Move Arm to dropCone Post, wrist level
        gamepadController.moveToAadiVector(dropConeAadiPose.getAadiVector(), dropConeAadiPose.getWristState());
        gamepadController.runArmShoulderWristToLevel();
        safeWait(750);

        //Open grip to drop Cone
        hand.openGrip();
        safeWait(750);

        //Move arm to neutral high, wrist level
        gamepadController.moveToNeutralHigh();
        safeWait(400);

        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    //Move subsysetms to a specific AadiPose
    public void moveToAadiPose(AadiPose aadiPose){
        arm.moveArmToLength(aadiPose.getArmLength());
        shoulder.moveShoulderToAngle(aadiPose.getShoulderAngle());
        if (aadiPose.getWristState() == Hand.WRIST_STATE.WRIST_LEVEL) {
            hand.moveWristLevel(aadiPose.getShoulderAngle());
        } else {
            hand.moveWristUp(aadiPose.getShoulderAngle());
        }
        turret.moveTurretToAngle(aadiPose.getTurretAngle());
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
                telemetry.addData("    Front High           ", "X / Square");
                telemetry.addData("    Back Medium          ", "A / Cross");
                telemetry.addData("    Back High            ", "B / Circle");
                if (gamepadController.gp1GetButtonXPress()) {
                    dropConePosition = DROP_CONE_POSITION.FRONT_HIGH;
                    break;
                }
                if (gamepadController.gp1GetButtonYPress()) {
                    dropConePosition = DROP_CONE_POSITION.BACK_MEDIUM;
                    break;
                }
                if (gamepadController.gp1GetButtonBPress()) {
                    dropConePosition = DROP_CONE_POSITION.BACK_HIGH;
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

        hand = new Hand(hardwareMap);
        telemetry.addLine("Hand Initialized");
        telemetry.update();

        arm = new Arm(hardwareMap);
        telemetry.addLine("Arm Initialized, Pulled in completely");
        telemetry.addData("  - Arm Touch Sensor State", arm.armTouchSensor.getState());
        telemetry.update();

        shoulder = new Shoulder(hardwareMap);
        telemetry.addLine("Shoulder Initialized, Pushed down completely");
        telemetry.addData("  - Should Touch Sensor State: ", shoulder.shoulderTouchSensor.getState());
        telemetry.update();

        turret = new Turret(hardwareMap);
        telemetry.addLine("Turret Initialized, Set to middle");
        telemetry.addData("  - Turret Left Mag Sensor State: ", turret.turretLeftMagneticSensor.getState());
        telemetry.addData("  - Turret Center Mag Sensor State: ", turret.turretCenterMagneticSensor.getState());
        telemetry.addData("  - Turret Right Mag Sensor State: ", turret.turretRightMagneticSensor.getState());
        telemetry.update();

        lights = new Lights(hardwareMap);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        vision = new Vision(hardwareMap);
        telemetry.addLine("Vision Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, arm, hand, shoulder, turret, lights);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        //GameField.debugLevel = GameField.DEBUG_LEVEL.NONE;
        GameField.debugLevel = GameField.DEBUG_LEVEL.MAXIMUM;

        telemetry.addLine("+++++++++++++++++++++++");
        telemetry.addLine("Init Completed, Select Starting Position");
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
