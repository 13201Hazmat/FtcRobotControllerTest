package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import static org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides.INTAKE_MOTOR_POWER_RESET;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.IntakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.util.Objects;

/**
 * FTC WIRES Autonomous Example
 */
@Autonomous(name = "Hazmat Auto 3", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutoOpMode3 extends LinearOpMode{

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

    public int dropConeCount = 6;
    public int stackConeCount = 5;

    public GamepadController gamepadController;
    public DriveTrain driveTrain;
    public IntakeArm intakeArm;
    public IntakeSlides intakeSlides;
    public OuttakeArm outtakeArm;
    public OuttakeSlides outtakeSlides;
    public Lights lights;
    public Vision vision;

    public ElapsedTime gameTimer = new ElapsedTime(MILLISECONDS);
    public ElapsedTime startTimer = new ElapsedTime(MILLISECONDS);
    public double startTime = 0;
    public ElapsedTime totalCyclingTimer = new ElapsedTime(MILLISECONDS);
    public double totalCyclingTime = 0;
    public ElapsedTime parkTimer = new ElapsedTime(MILLISECONDS);
    public double parkTime = 0;

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

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

        while (!isStopRequested() && !opModeIsActive()) {
            //Run Vuforia Tensor Flow and keep watching for the identifier in the Signal Cone.
            vision.runVuforiaTensorFlow();
            telemetry.clearAll();
            telemetry.addLine("Start Hazmat Autonomous Mode");
            telemetry.addData("---------------------------------------","");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("AutoOption Selected", autoOption);
            telemetry.addData("Vision identified Parking Location", vision.visionIdentifiedTarget);
            telemetry.addData("Drop Cone Count", dropConeCount);
            telemetry.update();
        }

        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {

            gameTimer.reset();
            startTimer.reset();
            //Turn Lights Green
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEFAULT);


            //Stop Vision process
            vision.deactivateVuforiaTensorFlow();

            //Build parking trajectory based on last detected target by vision
            buildParking();

            //Turn Auto Demo light patten on
            lights.setPattern(Lights.REV_BLINKIN_PATTERN.DEMO);

            //run Autonomous trajectory
            runAutoAndParking();
            parkTime = parkTimer.time();
        }

        safeWait(10000); // TODO for checking time.. need to remove
        telemetry.addData("Start Time", startTime);
        telemetry.addData("Total Cycling Time", cycleTime);
        telemetry.addData("Average Cycle time", averageCycleTime/(dropConeCount-0.5));
        telemetry.addData("parkTime", parkTime);
        //Trajectory is completed, display Parking complete
        parkingComplete();

        lights.setPattern(Lights.REV_BLINKIN_PATTERN.NONE);

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
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_HIGH_RIGHT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_MEDIUM_LEFT;
                break;

            case BLUE_RIGHT:
                initPose = new Pose2d(-64, -36, Math.toRadians(0));//Starting pose
                midWayPose = new Pose2d(-12, -36, Math.toRadians(0)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(-12, -39, Math.toRadians(-90));
                pickAndDropMediumPose = new Pose2d(-12, -33, Math.toRadians(-90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_HIGH_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_MEDIUM_RIGHT;
                break;

            case RED_LEFT:
                initPose = new Pose2d(64, -36, Math.toRadians(180));//Starting pose
                midWayPose = new Pose2d(12, -36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone, 180
                pickAndDropHighPose = new Pose2d(12, -39, Math.toRadians(270));
                pickAndDropMediumPose = new Pose2d(12, -33, Math.toRadians(270));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_HIGH_RIGHT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_MEDIUM_LEFT;
                break;

            case RED_RIGHT:
                initPose = new Pose2d(64, 36, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(15, 36, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(15, 39, Math.toRadians(90));
                pickAndDropMediumPose = new Pose2d(15, 33, Math.toRadians(90));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.AUTO_HIGH_LEFT;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.AUTO_MEDIUM_RIGHT;
                break;

            case TEST_POSE:
                initPose = new Pose2d(0, 0, Math.toRadians(180)); //Starting pose
                midWayPose = new Pose2d(3, 0, Math.toRadians(180)); //Choose the pose to move forward towards signal cone
                pickAndDropHighPose = new Pose2d(4, 0, Math.toRadians(180));
                pickAndDropMediumPose = new Pose2d(4, 0, Math.toRadians(180));
                pickAndDropTurretStateHigh= OuttakeSlides.TURRET_STATE.CENTER;
                pickAndDropTurretStateMedium = OuttakeSlides.TURRET_STATE.CENTER;
                parkPose = new Pose2d(5, 0, Math.toRadians(180));
                endPoseForward = 1;
                break;
        }

        if (autoOption == AUTO_OPTION.ONLY_PARK) {
            pickAndDropPose = pickAndDropMediumPose;
        }

        switch (dropConePosition) {
            case MEDIUM:
                outtakeDropState = OuttakeSlides.OUTTAKE_SLIDE_STATE.AUTO_MEDIUM_JUNCTION;
                pickAndDropPose = pickAndDropMediumPose;
                pickAndDropTurretState = pickAndDropTurretStateMedium;
                break;
            case HIGH:
                outtakeDropState = OuttakeSlides.OUTTAKE_SLIDE_STATE.AUTO_HIGH_JUNCTION;
                pickAndDropPose = pickAndDropHighPose;
                pickAndDropTurretState = pickAndDropTurretStateHigh;
                break;
        }

        //Move forward to midWayPose, rotate turret to 0 and reset turret

        trajectoryAuto = driveTrain.trajectorySequenceBuilder(initPose)
                .addTemporalMarker(0.1 ,() -> {
                    outtakeSlides.moveTurret(pickAndDropTurretState);
                })
                .lineToLinearHeading(midWayPose)
                /*
                .setVelConstraint(getVelocityConstraint(
                        0.5*DriveConstants.MAX_VEL,
                        0.5*DriveConstants.MAX_ANG_VEL,
                        DriveConstants.TRACK_WIDTH))
                 */
                .lineToLinearHeading(pickAndDropPose)
                //.resetVelConstraint()
                .build();


    }

    //Build parking trajectory based on target detected by vision
    public void buildParking(){
        switch (startPosition) {
            case BLUE_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, 60, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, 36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, 12, Math.toRadians(180)); break; // Location 3
                }
                endPoseTurn = 90;
                endPoseForward = 10;
                break;
            case BLUE_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(-12, -12, Math.toRadians(180)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(-12, -36, Math.toRadians(180)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(-12, -60, Math.toRadians(180)); break; // Location 3
                }
                endPoseTurn = -90;
                endPoseForward = 10;
                break;
            case RED_LEFT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(12, -60, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(12, -36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(12, -12, Math.toRadians(0)); break; // Location 3
                }
                endPoseTurn = 90;
                endPoseForward = 10;
                break;
            case RED_RIGHT:
                switch(vision.visionIdentifiedTarget){
                    case LOCATION1: parkPose = new Pose2d(15, 12, Math.toRadians(0)); break; // Location 1
                    case LOCATION2: parkPose = new Pose2d(15, 36, Math.toRadians(0)); break; // Location 2
                    case LOCATION3: parkPose = new Pose2d(15, 60, Math.toRadians(0)); break; // Location 3
                }
                endPoseTurn = -90;
                endPoseForward = 10;
                break;
            case TEST_POSE:
                break;
        }

        trajectoryParking = driveTrain.trajectorySequenceBuilder(pickAndDropPose)
                .lineToLinearHeading(parkPose)
                /*.addTemporalMarker(()-> {
                    telemetry.addData("Park Pose estimate", driveTrain.getPoseEstimate());
                    telemetry.update();
                })*/
                //.turn(Math.toRadians(endPoseTurn))
                //.forward(endPoseForward)
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

        if(opModeIsActive() && !isStopRequested() && startPosition != START_POSITION.TEST_POSE) {
            driveTrain.followTrajectorySequence(trajectoryAuto);
        }
        if (startPosition == START_POSITION.TEST_POSE) {
            outtakeSlides.moveTurret(pickAndDropTurretState);
            safeWait(1000);
        }
        startTime = startTimer.time();
        totalCyclingTimer.reset();

        //turn turret and pick, then drop cone
        if (autoOption != AUTO_OPTION.ONLY_PARK) {
            autoPickAndDropStateMachine();
        }

        totalCyclingTime = totalCyclingTimer.time();
        parkTimer.reset();

        ElapsedTime exitTimer = new ElapsedTime(MILLISECONDS);

        //End Condition for Auto
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
        exitTimer.reset();
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        switch (startPosition) {
            case BLUE_LEFT:
            case RED_LEFT:
                outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.TELEOP_LEFT);
                break;
            case BLUE_RIGHT:
            case RED_RIGHT:
                outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.TELEOP_RIGHT);
                break;
            case TEST_POSE:
                outtakeSlides.moveTurret(OuttakeSlides.TURRET_STATE.CENTER);
                break;
        }
        while (opModeIsActive() && !isStopRequested() &&
                exitTimer.time()<1000 && intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)) {
           safeWait(100);
        }
        outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);

        if(opModeIsActive() && !isStopRequested() && startPosition != START_POSITION.TEST_POSE) {
            driveTrain.followTrajectorySequence(trajectoryParking);
        }
        safeWait(1000);
    }

    public enum INTAKE_STATE{
        I0, I1, I2, I3, I4, I5, I6, I7, I8, I9, I10, I11, I12, I13, I14, I15;
    }
    public INTAKE_STATE intakeState = INTAKE_STATE.I0;

    public enum OUTTAKE_STATE{
        O0, O1, O2, O3, O4, O5, O6, O7, O8, O9, O10, O11;
    }
    public OUTTAKE_STATE outtakeState = OUTTAKE_STATE.O0;

    ElapsedTime intakeGripTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime intakeArmTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime outtakeWristTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime outtakeGripTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime outtakeSenseTimer = new ElapsedTime(MILLISECONDS);
    ElapsedTime cycleTimer = new ElapsedTime(MILLISECONDS);
    public double cycleTime = 0;
    public double averageCycleTime = 0;
    public int dropConeCounter = 0;
    public int stackConeCounter = 0;
    public int stateMachineLoopCounter = 0;

    public void autoPickAndDropStateMachine(){
        telemetry.setAutoClear(true);
        cycleTimer.reset();
        while(opModeIsActive() && !isStopRequested() &&
                dropConeCounter < dropConeCount ) {
            /*if ((outtakeState == OUTTAKE_STATE.O1 && intakeState == INTAKE_STATE.I1) //TODO : TO BE FIXED TO NEW SYNC POINT
                 && gameTimer.time() > 22000 && startPosition != START_POSITION.TEST_POSE) {
                break;
            }*/
            stateMachineLoopCounter ++;
            switch (outtakeState) {
                case O0: // Start of State machine
                    outtakeState = OUTTAKE_STATE.O1;
                    break;

                case O1: // Cone is sensed, close grip
                    if (dropConeCounter == 0) {
                        outtakeArm.closeGrip();
                        outtakeState = OUTTAKE_STATE.O3;
                    } else if (intakeState == INTAKE_STATE.I12) {
                        outtakeArm.closeGrip();
                        outtakeGripTimer.reset();
                        outtakeState = OUTTAKE_STATE.O2;
                    }
                    break;

                case O2: // If Intake Arm is not in transfer, proceed to move outttake to drop position
                    /*telemetry.addData("!intakeArm.isIntakeArmInTransfer()",
                            !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER));*/
                    if(intakeState == INTAKE_STATE.I1 && outtakeGripTimer.time() > 200 &&
                            !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O3;
                    }
                    break;

                case O3: // Move outtake to drop Position
                    outtakeSlides.moveOuttakeSlides(outtakeDropState);
                    if (dropConePosition == DROP_CONE_POSITION.HIGH) {
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.AUTO_HIGH_JUNCTION);
                    } else { // dropConePosition == DROP_CONE_POSITION.MEDIUM
                        outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.AUTO_MEDIUM_JUNCTION);
                    }
                    outtakeWristTimer.reset();
                    outtakeState = OUTTAKE_STATE.O4;
                    break;

                case O4: // Move outtake wrist to Drop
                    if (outtakeWristTimer.time() > 300) {
                        outtakeArm.moveWrist(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_AUTO_DROP);
                        outtakeState = OUTTAKE_STATE.O5;
                        outtakeWristTimer.reset();
                    }
                    break;

                case O5: // Verify that Outtake slide, Arm and Wrist are in position to drop
                    /*telemetry.addData("isOuttakeSlidesInState(outtakeDropState)", outtakeSlides.isOuttakeSlidesInState(outtakeDropState));
                    telemetry.addData("isOuttakeSlidesInStateError", outtakeSlides.isOuttakeSlidesInStateError);
                    telemetry.addData("outtakeMotor.getCurrentPosition()", outtakeSlides.outtakeMotor.getCurrentPosition());
                    telemetry.addData("outtakeDropState.motorPosition", outtakeDropState.motorPosition);
                    telemetry.addData("isOuttakeArmInState(DROP)", outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP));
                    telemetry.addData("isOuttakeArmInStateError", outtakeArm.isOuttakeArmInStateError);
                    telemetry.addData("isOuttakeWristInState(DROP)", outtakeArm.isOuttakeWristInState(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_DROP));
                    telemetry.addData("isOuttakeWristInStateError", outtakeArm.isOuttakeWristInStateError);
                    telemetry.addData("outtakeWristServo.getPosition()", outtakeArm.outtakeWristServo.getPosition());
                    telemetry.addData("WRIST_DROP position",OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_DROP.getWristPosition() );*/

                    if ((outtakeWristTimer.time() > 500 &&
                            outtakeSlides.isOuttakeSlidesInState(outtakeDropState)
                            && outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.DROP)
                            && outtakeArm.isOuttakeWristInState(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_AUTO_DROP))
                            || outtakeWristTimer.time() > 1000) {//750
                        outtakeState = OUTTAKE_STATE.O6;
                    }
                    break;

                case O6: // Open Grip and Drop Cone
                    outtakeArm.openGrip();
                    outtakeGripTimer.reset();
                    outtakeState = OUTTAKE_STATE.O7;
                    break;

                case O7: // Wait for cone to be dropped completely
                    if (outtakeGripTimer.time() > 400) {
                        outtakeState = OUTTAKE_STATE.O8;
                    }
                    break;

                case O8: // Check if intake is in Transfer, else Move outtake to transfer
                    /*telemetry.addData("!intakeArm.isIntakeArmInTransfer()",
                            !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER));*/
                    if (!intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O9;
                    }
                    break;

                case O9: // Move outtake to Transfer
                    outtakeArm.moveArm(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER);
                    outtakeSlides.moveOuttakeSlides(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER);
                    outtakeArm.openGrip();
                    dropConeCounter++;
                    outtakeState = OUTTAKE_STATE.O10;

                case O10: // Check if outtake is at transfer
                    //telemetry.addData("outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)", outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER));
                    //telemetry.addData("outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)", outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER));
                    if(outtakeArm.isOuttakeArmInState(OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER)
                            && outtakeArm.isOuttakeWristInState(OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER)
                            && outtakeArm.outtakeGripState == OuttakeArm.OUTTAKE_GRIP_STATE.OPEN
                            && outtakeSlides.isOuttakeSlidesInState(OuttakeSlides.OUTTAKE_SLIDE_STATE.TRANSFER)) {
                        outtakeState = OUTTAKE_STATE.O11;
                    }
                    break;

                case O11: // Wait till Intake drops cone
                    //telemetry.addData("outtakeArm.senseOuttakeCone()", outtakeArm.senseOuttakeCone());
                    if (dropConeCounter < dropConeCount) {
                        if (intakeState == INTAKE_STATE.I12) {
                            if (outtakeArm.senseOuttakeCone() || outtakeSenseTimer.time() > 500) { //1500
                                outtakeState = OUTTAKE_STATE.O1;
                                cycleTime = cycleTimer.time();
                                averageCycleTime +=cycleTime;
                                cycleTimer.reset();
                            }
                        }
                    } else {
                        outtakeState = OUTTAKE_STATE.O1;
                        cycleTime = cycleTimer.time();
                        averageCycleTime +=cycleTime;
                        cycleTimer.reset();
                    }
                    break;
            }

            switch (intakeState) {
                case I0: // Start of State Machine
                    intakeState = INTAKE_STATE.I1;
                    break;

                case I1: // Start fo next round of intake sequence, if stack is not empty
                         // if only drop preloaded, Intake state machine wont move forward
                    if(stackConeCounter < stackConeCount) {
                        intakeState = INTAKE_STATE.I2;
                    }
                    break;

                case I2: //Move Intake arm and slides to stack current cone level
                    intakeArm.moveArm(Objects.requireNonNull(intakeArm.intakeArmState.byIndex(5 - stackConeCounter)));

                    intakeSlides.moveIntakeSlides(Objects.requireNonNull(intakeSlides.intakeSlidesState.byIndex(5 - stackConeCounter)));
                    intakeSlides.runIntakeMotorToLevel();
                    intakeGripTimer.reset();
                    intakeState = INTAKE_STATE.I3;
                    break;

                case I3: // Wait for Outtake grip to be open, and hold position minimum for 500 to stabilize
                    if( (intakeSlides.isIntakeSlidesInState(intakeSlides.intakeSlidesState.byIndex(5 - stackConeCounter))
                            && outtakeArm.isOuttakeGripInState(OuttakeArm.OUTTAKE_GRIP_STATE.OPEN)
                            && intakeGripTimer.time() > 300)
                            || intakeGripTimer.time() > 1000) {//400
                        intakeState = INTAKE_STATE.I4;
                    }

                    break;

                case I4: // Close grip on stack
                    intakeArm.closeGrip();
                    intakeGripTimer.reset();
                    intakeState = INTAKE_STATE.I5;
                    break;

                case I5 : // Wait for grip to be closed completely
                    if (intakeGripTimer.time() > 400 && intakeArm.isIntakeGripInState(IntakeArm.INTAKE_GRIP_STATE.CLOSED)) {
                        intakeState = INTAKE_STATE.I6;
                    }
                    break;

                case I6: // Move Arm up to clear stack
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.LOW_JUNCTION);
                    intakeArmTimer.reset();
                    intakeState = INTAKE_STATE.I7;
                    break;

                case I7: // Wait for Arm to be completely moved up of stack
                    if (intakeArmTimer.time() > 300) {
                        intakeState = INTAKE_STATE.I8;
                    }
                    break;
                case I8: // Move intake slides to Transfer
                    intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
                    intakeState = INTAKE_STATE.I9;
                    break;

                case I9: // Wait till Outtake is ready to accept cone
                    if(outtakeState == OUTTAKE_STATE.O11){
                        intakeState = INTAKE_STATE.I10;
                    }
                    break;

                case I10: // Move intake Arm to transfer
                    intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
                    intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.TRANSFER);
                    intakeArmTimer.reset();
                    intakeState = INTAKE_STATE.I11;
                    break;

                case I11: // Check if intake Arm and Slides are in Transfer
                    //telemetry.addData("intakeArm.isIntakeArmInTransfer", intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER) );
                    //telemetry.addData("intakeSlides.isIntakeSlidesInTransfer", intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER));

                    if((intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.TRANSFER)
                            && intakeSlides.isIntakeSlidesInState(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER))
                            || intakeArmTimer.time() > 1000) {
                        outtakeSenseTimer.reset();
                        intakeState = INTAKE_STATE.I12;
                    }
                    break;

                case I12: // Open Intake Grip to drop cone to Transfer
                    if (outtakeState == OUTTAKE_STATE.O2) {
                        intakeArm.openGrip();
                        safeWait(200);
                        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
                        intakeArmTimer.reset();
                        while (intakeArmTimer.time() < 700 &&
                                !intakeArm.isIntakeArmInState(IntakeArm.INTAKE_ARM_STATE.INIT)) {}
                        stackConeCounter++;
                        intakeState = INTAKE_STATE.I1;
                    }
                    break;
            }
            telemetry.addData("dropConeCounter", dropConeCounter);
            telemetry.addData("stackConeCounter", stackConeCounter);
            telemetry.addData("cycleTime", cycleTime);
            telemetry.addData(" --- OuttakeState", outtakeState);
            telemetry.addData(" --- IntakeState", intakeState);

            telemetry.update();
            if (startPosition == START_POSITION.TEST_POSE) {
                safeWait(0);
            }

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
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
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
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
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



        if (autoOption == AUTO_OPTION.ONLY_PARK) {
            telemetry.addLine("Initializing Hazmat Autonomous Mode ");
            telemetry.addData("---------------------------------------", "");
            telemetry.addData("Selected Starting Position", startPosition);
            telemetry.addData("Selected Auto Option", autoOption);
            dropConeCount = 0;
            stackConeCount = 0;
        } else {
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
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
            dropConeCount = 6;
            while (!isStopRequested()) {
                telemetry.addLine("Initializing Hazmat Autonomous Mode ");
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
        intakeSlides.moveIntakeSlides(IntakeSlides.INTAKE_SLIDES_STATE.TRANSFER);
        telemetry.addLine("IntakeSlides Initialized");
        telemetry.update();

        intakeArm = new IntakeArm(hardwareMap);
        intakeArm.moveArm(IntakeArm.INTAKE_ARM_STATE.INIT);
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