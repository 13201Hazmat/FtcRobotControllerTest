package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.AadiGeometry.AadiVector;
import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.AadiGeometry.AadiPose;
import org.firstinspires.ftc.teamcode.SubSystems.Arm;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Shoulder;
import org.firstinspires.ftc.teamcode.SubSystems.SystemState;
import org.firstinspires.ftc.teamcode.SubSystems.Turret;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

/**
 * FTC WIRES Autonomous Example
 */
@Autonomous(name = "Hazmat Autonomous", group = "00-Autonomous", preselectTeleOp = "FTC Wires TeleOp")
public class AutoOpMode extends LinearOpMode{

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public Vision vision;
    public DriveTrain driveTrain;
    public Hand hand;
    public Arm arm;
    public Shoulder shoulder;
    public Turret turret;
    public Lights lights;
    public GamepadController gamepadController;

    public ElapsedTime gameTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);

    @Override
    public void runOpMode() throws InterruptedException {
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
            telemetry.addData("Start FTC Wires (ftcwires.org) Autonomous Mode adopted for Team","TEAM NUMBER");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
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
    AadiPose initAadiPose;
    AadiPose[] pickConeAadiPose = new AadiPose[6];
    AadiPose[] dropConeAadiPose = new AadiPose[6];
    AadiPose endAadiPose;

    int coneCount = 0;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConeAadiPose[0] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(300,280, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[1] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[2] = new AadiPose(300,210, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[2] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[3] = new AadiPose(300,140, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[3] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[4] = new AadiPose(300,70, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[4] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[5] = new AadiPose(300,0, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[5] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                dropConeAadiPose[0] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(300,280, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[1] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[2] = new AadiPose(300,210, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[2] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[3] = new AadiPose(300,140, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[3] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[4] = new AadiPose(300,70, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[4] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[5] = new AadiPose(300,0, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[5] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                break;

            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                dropConeAadiPose[0] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(300,280, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[1] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[2] = new AadiPose(300,210, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[2] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[3] = new AadiPose(300,140, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[3] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[4] = new AadiPose(300,70, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[4] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                pickConeAadiPose[5] = new AadiPose(300,0, Hand.WRIST_STATE.WRIST_UP, -675);
                dropConeAadiPose[5] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, -1700);
                endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                break;

            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                initAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                dropConeAadiPose[0] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(300,280, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[1] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[2] = new AadiPose(300,210, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[2] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[3] = new AadiPose(300,140, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[3] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[4] = new AadiPose(300,70, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[4] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                pickConeAadiPose[5] = new AadiPose(300,0, Hand.WRIST_STATE.WRIST_UP, 675);
                dropConeAadiPose[5] = new AadiPose(1460,760, Hand.WRIST_STATE.WRIST_UP, 1700);
                endAadiPose = new AadiPose(0,shoulder.MAX_RAISED_POSITION, Hand.WRIST_STATE.WRIST_UP, 0);
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .addDisplacementMarker(() -> {moveToAadiPose(initAadiPose);})
                .lineToLinearHeading(midWayPose)
                //turn turret and pick, then drop cone
                .addDisplacementMarker(() -> {
                    dropCone(dropConeAadiPose[0]); //Drop preloaded Cone
                    for (coneCount = 1; coneCount <=5; coneCount++) {
                        pickCone(pickConeAadiPose[coneCount]);
                        dropCone(dropConeAadiPose[coneCount]);
                    }
                    moveToAadiPose(endAadiPose);
                })
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;
            case BLUE_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;
            case RED_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;
            case RED_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
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
        telemetry.addData("Running FTC Wires (ftcwires.org) Autonomous Mode adopted for Team:","TEAM NUMBER");
        telemetry.addData("---------------------------------------","");
        telemetry.update();
        //Run the trajectory built for Auto and Parking
        driveTrain.followTrajectorySequence(trajectoryAuto);
        driveTrain.followTrajectorySequence(trajectoryParking);
    }

    ElapsedTime pickTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(AadiPose pickConeAadiPose) {
        hand.openGrip();
        pickTimer.reset();
        gamepadController.moveToAadiVector(SystemState.NEUTRAL_VECTOR);
        while (pickTimer.time() < 1000) {
            gamepadController.runArmShoulderWristToLevel();
        };
        pickTimer.reset();
        turret.moveTurretToAngle(pickConeAadiPose.getTurretAngle());
        while (pickTimer.time() < 1000) {
            gamepadController.runTurret();
        };
        gamepadController.moveToAadiVector(pickConeAadiPose.getAadiVector());
        gamepadController.runArmShoulderWristToLevel();
        hand.closeGrip();
        shoulder.moveShoulderToAngle(shoulder.shoulderCurrentPosition + 20);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    ElapsedTime dropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(AadiPose dropConeAadiPose){
        dropTimer.reset();
        gamepadController.moveToAadiVector(SystemState.NEUTRAL_VECTOR);
        while (pickTimer.time() < 1000) {
            gamepadController.runArmShoulderWristToLevel();
        };
        pickTimer.reset();
        turret.moveTurretToAngle(dropConeAadiPose.getTurretAngle());
        while (pickTimer.time() < 1000) {
            gamepadController.runTurret();
        };
        gamepadController.moveToAadiVector(dropConeAadiPose.getAadiVector());
        gamepadController.runArmShoulderWristToLevel();
        hand.openGrip();
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
            telemetry.addData("    Blue Left   ", "(X)");
            telemetry.addData("    Blue Right ", "(Y)");
            telemetry.addData("    Red Left    ", "(B)");
            telemetry.addData("    Red Right  ", "(A)");
            if(gamepadController.gp1GetButtonXPress()){
                startPosition = START_POSITION.BLUE_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonYPress()){
                startPosition = START_POSITION.BLUE_RIGHT;
                break;
            }
            if(gamepadController.gp1GetButtonBPress()){
                startPosition = START_POSITION.RED_LEFT;
                break;
            }
            if(gamepadController.gp1GetButtonAPress()){
                startPosition = START_POSITION.RED_RIGHT;
                break;
            }
            telemetry.update();
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


}
