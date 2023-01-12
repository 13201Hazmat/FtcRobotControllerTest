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

import java.util.Objects;

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
        RED_RIGHT,
        TEST_POSE
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

    public int dropConeCount = 0;
    public int stackConeCount = 0;

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

            gameTimer.reset();
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
    double endPoseTurn;
    double endPoseForward;

    OuttakeSlides.OUTTAKE_SLIDE_STATE outtakeDropState;

    OuttakeSlides.TURRET_STATE pickAndDropTurretStateHigh;
    OuttakeSlides.TURRET_STATE pickAndDropTurretStateMedium;
    OuttakeSlides.TURRET_STATE pickAndDropTurretState;

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
                pickAndDropHighPose = new Pose2d(12, -39, Math.toRadians(90));
                pickAndDropMediumPose = new Pose2d(12, -33, Math.toRadians(90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                break;

            case RED_RIGHT:
                initPose = new Pose2d(64, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(12, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(12, 39, Math.toRadians(-90));
                pickAndDropMediumPose = new Pose2d(12, 33, Math.toRadians(-90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                break;

            case TEST_POSE:
                initPose = new Pose2d(0, 0, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(3, 0, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(4, 0, Math.toRadians(180));
                pickAndDropMediumPose = new Pose2d(4, 0, Math.toRadians(180));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_RIGHT;
                parkPose = new Pose2d(5, 0, Math.toRadians(180));
                endPoseForward = 1;
                break;
        }

        if (autoOption == AUTO_OPTION.ONLY_PARK) {
            pickAndDropPose = pickAndDropMediumPose;
        }

        switch (dropConePosition) {
            case MEDIUM:
                outtakeDropState = OuttakeSlides.OUTTAKE_SLIDE_STATE.MEDIUM_JUNCTION;
                pickAndDropPose = pickAndDropMediumPose;
                pickAndDropTurretState = pickAndDropTurretStateMedium;
                break;
            case HIGH:
                outtakeDropState = OuttakeSlides.OUTTAKE_SLIDE_STATE.HIGH_JUNCTION;
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
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(90)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(90)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 12, Math.toRadians(90)); break; // Location 3
                }
                endPoseTurn = 90;
                endPoseForward = 20;
                break;
            case BLUE_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -12, Math.toRadians(-90)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(-90)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(-90)); break; // Location 3
                }
                endPoseTurn = -90;
                endPoseForward = 20;
                break;
            case RED_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(90)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(90)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -12, Math.toRadians(90)); break; // Location 3
                }
                endPoseTurn = 90;
                endPoseForward = 20;
                break;
            case RED_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, 12, Math.toRadians(90)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, 36, Math.toRadians(90)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, 60, Math.toRadians(90)); break; // Location 3
                }
                endPoseTurn = -90;
                endPoseForward = 20;
                break;
            case TEST_POSE:
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(pickAndDropPose)
                .lineToLinearHeading(parkPose)
                .turn(Math.toRadians(endPoseTurn))
                .forward(endPoseForward)
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
            telemetry.addData("Selected number of dropCones including preloaded", dropConeCount);
        }
        telemetry.update();

        //Move forward to midWayPose, rotate turret to 0 and reset turret.

        if(opModeIsActive() && !isStopRequested()) {
            driveTrain.followTrajectorySequence(trajectoryAuto);
        }
        outtakeSlides.moveTurret(pickAndDropTurretState);

        //turn turret and pick, then drop cone
        if (autoOption != AUTO_OPTION.ONLY_PARK) {
            autoPickAndDropStateMachine();
        }

        ElapsedTime exitTimer = new ElapsedTime(MILLISECONDS);

        //End Condition for Auto
        intakeArm.moveArm(IntakeArm.ARM_STATE.INIT);
        exitTimer.reset();
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_MOTOR_STATE.TRANSFER);
        outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.CENTER);
        while (opModeIsActive() && !isStopRequested() &&
                exitTimer.time()<1000 && intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER)) {
           safeWait(100);
        }
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);

        if(opModeIsActive() && !isStopRequested()) {
            driveTrain.followTrajectorySequence(trajectoryParking);
        }

        safeWait(1000);
    }

    public enum INTAKE_STATE{
        I0, I1, I2, I3, I4, I5, I6, I7, I8, I9;
    }
    public INTAKE_STATE intakeState = INTAKE_STATE.I0;

    public enum OUTTAKE_STATE{
        O0, O1, O2, O3, O4, O5, O6, O7, O8, O9, O10;
    }
    public OUTTAKE_STATE outtakeState = OUTTAKE_STATE.O0;

    ElapsedTime intakeGripTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime outtakeGripTimer = new ElapsedTime(MILLISECONDS);
    public int dropConeCounter = 0;
    public int stackConeCounter = 0;

    public void autoPickAndDropStateMachine(){
        while(opModeIsActive() && !isStopRequested() &&
                dropConeCounter != dropConeCount /*&&
                gameTimer.time() < 26000*/) {
            telemetry.addLine("Beginning of While loop");
            telemetry.addData("dropConeCounter", dropConeCounter);
            telemetry.addData("dropConeCount", dropConeCount);
            telemetry.addData("stackConeCounter", stackConeCounter);
            telemetry.addData("stackConeCount", stackConeCount);
            //safeWait( 1000); // For Testing

            switch (outtakeState) {
                case O0:
                    outtakeState = OUTTAKE_STATE.O1;
                    break;

                case O1:
                    outtakeArm.closeGrip();
                    outtakeState = OUTTAKE_STATE.O2;
                    break;

                case O2:
                    telemetry.addData("intakeArm.isIntakeArmInTransfer()",
                            intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER));
                    if(!intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O3;
                    }
                    break;

                case O3:
                    outtakeSlides.moveOuttakeSlides(outtakeDropState);
                    outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.DROP);
                    outtakeState = OUTTAKE_STATE.O4;
                    break;

                case O4:
                    telemetry.addData("outtakeSlides.isOuttakeSlidesInState(outtakeDropState)", outtakeSlides.isOuttakeSlidesInState(outtakeDropState));
                    telemetry.addData("outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP)", outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP));
                    telemetry.addData("outtakeArm.outtakeArmLeft.getPosition()", outtakeArm.outtakeArmLeft.getPosition());
                    if (outtakeSlides.isOuttakeSlidesInState(outtakeDropState) &&
                            outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP)) {
                        outtakeState = OUTTAKE_STATE.O5;
                    }
                    break;

                case O5:
                    outtakeArm.openGrip();
                    dropConeCount ++;
                    outtakeGripTimer.reset();
                    outtakeState = OUTTAKE_STATE.O6;
                    break;

                /*case O6:
                    if (outtakeGripTimer.time() > 200) {
                        dropConeCount ++;
                        outtakeState = OUTTAKE_STATE.O7;
                    }
                    break;*/

                case O6:
                    telemetry.addData("!intakeArm.isIntakeArmInTransfer()",
                            !intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER));
                    if (!intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O7;
                    }
                    break;

                case O7:
                    outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                    outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
                    outtakeArm.openGrip();
                    outtakeState = OUTTAKE_STATE.O8;

                case O8:
                    telemetry.addData("outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)", outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER));
                    telemetry.addData("outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)", outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER));
                    if(outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)
                            && outtakeArm.isOuttakeWristInState(OuttakeArm.WRIST_STATE.WRIST_TRANSFER)
                            && outtakeArm.gripState == OuttakeArm.GRIP_STATE.OPEN
                            && outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O9;
                    }
                    break;

                case O9:
                    telemetry.addData("outtakeArm.senseOuttakeCone()", outtakeArm.senseOuttakeCone());
                    if(outtakeArm.senseOuttakeCone()) {
                        outtakeState = OUTTAKE_STATE.O1;
                    }
                    break;
            }

            switch (intakeState) {
                case I0:
                    if(stackConeCounter < stackConeCount) {
                        intakeState = INTAKE_STATE.I1;
                    }
                    break;

                case I1:
                    intakeArm.moveArm(Objects.requireNonNull(intakeArm.armState.byIndex(5 - stackConeCounter)));
                    intakeSlides.moveIntakeSlides(Objects.requireNonNull(intakeSlides.intakeSlidesState.byIndex(5 - stackConeCounter)));
                    intakeSlides.runIntakeMotorToLevel();
                    intakeState = INTAKE_STATE.I2;
                    break;

                case I2:
                    if(outtakeState == OUTTAKE_STATE.O6) {
                        intakeState = INTAKE_STATE.I3;
                    }
                    break;

                case I3:
                    intakeArm.closeGrip();
                    //intakeArm.moveWristUp(); // TODO : TEST WITH MOVE TO LOW JUNCTION
                    intakeArm.moveArm(IntakeArm.ARM_STATE.LOW_JUNCTION);
                    safeWait(300); // TODO : TEST WITH MOVE TO LOW JUNCTION
                    stackConeCounter = (stackConeCounter+1 < stackConeCount) ? stackConeCounter++ : stackConeCounter;
                    intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_MOTOR_STATE.TRANSFER);
                    intakeSlides.runIntakeMotorToLevel();
                    intakeState = INTAKE_STATE.I4;
                    break;

                case I4:
                    if(outtakeState == OUTTAKE_STATE.O9){
                        intakeState = INTAKE_STATE.I5;
                    }
                    break;

                case I5:
                    intakeArm.moveArm(IntakeArm.ARM_STATE.TRANSFER);
                    intakeState = INTAKE_STATE.I6;
                    break;

                case I6:
                    telemetry.addData("intakeArm.isIntakeArmInTransfer", intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER) );
                    telemetry.addData("intakeSlides.isIntakeSlidesInTransfer", intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_MOTOR_STATE.TRANSFER));

                    if(intakeArm.isIntakeArmInState(IntakeArm.ARM_STATE.TRANSFER)
                            && intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_MOTOR_STATE.TRANSFER)) {
                        intakeState = INTAKE_STATE.I7;
                    }
                    break;

                case I7:
                    intakeArm.openGrip();
                    intakeGripTimer.reset();
                    intakeState = INTAKE_STATE.I8;
                    break;

                case I8:
                    if (intakeGripTimer.time() > 500) {
                        intakeArm.moveArm(IntakeArm.ARM_STATE.INIT);
                        intakeState = INTAKE_STATE.I1;
                    }
                    break;
            }

            telemetry.addData("stackConeCounter", stackConeCounter);
            telemetry.addData("dropConeCounter", dropConeCounter);
            telemetry.addData(" IntakeState", intakeState);
            telemetry.addData(" OuttakeState", outtakeState);
            telemetry.update();

        }
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
            telemetry.addData("      TEST POSE", "Right Bumper");
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
            if (gamepadController.gp1GetRightBumperPress()){
                startPosition = START_POSITION.TEST_POSE;
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
                dropConeCount = 1;
                stackConeCount = 0;
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

                telemetry.addData("    dropConeCount (including preloaded, Max 6)", dropConeCount);

                if (gamepadController.gp1GetDpad_upPress()) {
                    dropConeCount += 1;
                    if (dropConeCount > 6) {
                        dropConeCount = 6;
                    }
                }
                if (gamepadController.gp1GetDpad_downPress()) {
                    dropConeCount -= 1;
                    if (dropConeCount < 1) {
                        dropConeCount = 1;
                    }
                }

                stackConeCount = dropConeCount -1;

                if (gamepadController.gp1GetButtonXPress()) {
                    telemetry.addData("Selected number of dropCones including preloaded", dropConeCount);
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

        intakeSlides = new IntakeSlides(hardwareMap);
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_MOTOR_STATE.TRANSFER);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap);
        intakeArm.moveArm(IntakeArm.ARM_STATE.INIT);
        intakeArm.openGrip();
        telemetry.addLine("IntakeArm Initialized");
        telemetry.update();

        outtakeSlides = new OuttakeSlides(hardwareMap);
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.MAX_LEFT);
        telemetry.addLine("OuttakeSlides Initialized");
        telemetry.update();

        outtakeArm = new OuttakeArm(hardwareMap);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
        outtakeArm.closeGrip();
        telemetry.addLine("OuttakeArm Initialized");
        telemetry.update();

        lights = new Lights(hardwareMap);
        telemetry.addLine("Lights Initialized");
        telemetry.update();

        /* Create Controllers */
        gamepadController = new GamepadController(gamepad1, gamepad2, driveTrain, intakeArm, intakeSlides, outtakeArm, outtakeSlides, lights);
        telemetry.addLine("Gamepad Initialized");
        telemetry.update();

        vision = new Vision(hardwareMap);
        telemetry.addLine("Vision Initialized");
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