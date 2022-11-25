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
    AadiPose midAadiPose;
    AadiPose[] dropConeAadiPose = new AadiPose[6];
    AadiPose endAadiPose;

    int coneCount = 0;

    //Set all position based on selected staring location and Build Autonomous Trajectory
    public void buildAuto() {
        switch (startPosition) {
            case BLUE_LEFT:
                initPose = new Pose2d(-54, 36, Math.toRadians(0)); //Starting pose
                initAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                midWayPose = new Pose2d(-12, 36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                midAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[0] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                endAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 11, Math.toRadians(180)); break; // Location 3
                }
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-54, -36, Math.toRadians(0));//Starting pose
                initAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                midAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[0] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                endAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -11, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                break;

            case RED_LEFT:
                initPose = new Pose2d(54, -36, Math.toRadians(180));//Starting pose
                initAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                midAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[0] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                endAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -11, Math.toRadians(0)); break; // Location 3
                }
                break;

            case RED_RIGHT:
                initPose = new Pose2d(54, 36, Math.toRadians(180)); //Starting pose
                initAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                midAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[0] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0); // Preloaded Cone drop position
                pickConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[1] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[2] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[3] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[4] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                pickConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                dropConeAadiPose[5] = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                endAadiPose = new AadiPose(0,0, AadiVector.WRIST_ANGLE.UP, 0);
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, 11, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, 60, Math.toRadians(0)); break; // Location 3
                }
                break;
        }

        //Drop Preloaded Cone, Pick 5 cones and park
        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .lineToLinearHeading(midWayPose)
                //turn turret and pick, then drop cone
                .addDisplacementMarker(() -> {
                    dropCone(dropConeAadiPose[0]); //Drop preloaded Cone
                    for (coneCount = 1; coneCount <=5; coneCount++) {
                        pickCone(pickConeAadiPose[coneCount]);
                        moveToAadiPose(midAadiPose);
                        dropCone(dropConeAadiPose[coneCount]);
                    }
                    moveToAadiPose(endAadiPose);
                })
                .build();
    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
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

    //Write a method which is able to pick the cone from the stack depending on your subsystems
    public void pickCone(AadiPose pickConeAadiPose) {
        /*TODO: Add code to pick Cone 1 from stack*/
        hand.openGrip();
        moveToAadiPose(pickConeAadiPose);
        hand.closeGrip();
        liftArmAfterPickCone(pickConeAadiPose);
        telemetry.addData("Picked Cone: Stack", coneCount);
        telemetry.update();
    }

    //Write a method which is able to drop the cone depending on your subsystems
    public void dropCone(AadiPose dropConeAadiPose){
        /*TODO: Add code to drop cone on junction*/
        if (coneCount == 0) {
            telemetry.addData("Dropped Cone", "Pre-loaded");
        } else {
            telemetry.addData("Dropped Cone: Stack", coneCount);
        }
        telemetry.update();
    }

    //Move subsysetms to a specific AadiPose
    public void moveToAadiPose(AadiPose aadiPose){
        arm.moveArmToLength(aadiPose.getArmLength(aadiPose));
        shoulder.moveShoulderToAngle(aadiPose.getShoulderAngle(aadiPose));
        if (aadiPose.getWristAngle(aadiPose) == AadiVector.WRIST_ANGLE.LEVEL) {
            hand.moveWristLevel(aadiPose.getShoulderAngle(aadiPose));
        } else {
            hand.moveWristUp(aadiPose.getShoulderAngle(aadiPose));
        }
        turret.moveTurretToAngle(aadiPose.getTurretAngle(aadiPose));
    }

    //Lift Arm to clear cone from Stack before moving to next position TODO
    public void liftArmAfterPickCone(AadiPose aadiPose){
        //Lift arm to move cone above stack
    }

    public AadiPose createAadiPoseFromWorld(double armMM, double shoulderRadians, AadiVector.WRIST_ANGLE wristAngle, double turretRadians) {
        return new AadiPose(AadiPose.armMMtoLength(armMM),
                AadiPose.shoulderRadiansToAngle(shoulderRadians),
                wristAngle,
                AadiPose.turretRadiansToAngle(turretRadians));
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
