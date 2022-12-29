package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class IntakeSlides {

    //Initialization of intakemotor
    public DcMotorEx intakeMotorLeft, intakeMotorRight;

    //Intake Motor : 5203 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 8mm REX Shaft, 435 RPM, 3.3 - 5V Encoder)
    public static final double INTAKE_MOTOR_ENCODER_TICKS = 1500; //384.5

    public DigitalChannel intakeTouch;  // Hardware Device Object

    public DistanceSensor intakeDistanceSensor;

    //Intake Motor states
    public enum INTAKE_MOTOR_STATE {
        MIN_RETRACTED (0), //Position
        TRANSFER (0),
        MAX_EXTENDED(0),
        RANDOM(0),
        
        AUTO_CONE_1(0),
        AUTO_CONE_2(0),
        AUTO_CONE_3(0),
        AUTO_COME_4(0),
        AUTO_CONE_5(0);

        private final double motorPosition;
        INTAKE_MOTOR_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public IntakeSlides.INTAKE_MOTOR_STATE intakeMotorState = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED;

    public double intakeMotorCurrentPosition = intakeMotorState.motorPosition;
    public double intakeMotorNewPosition = intakeMotorState.motorPosition;

    public static final double INTAKE_MOTOR_DELTA_COUNT_MAX = 200;//200;//200 //need tested values
    public static final double INTAKE_MOTOR_DELTA_COUNT_RESET = 50;

    //Different constants of arm speed
    public static final double INTAKE_MOTOR_POWER_TELEOP = 0.8;
    public static final double INTAKE_MOTOR_POWER_AUTO = 0.8;
    public enum INTAKE_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }
    public IntakeSlides.INTAKE_MOVEMENT_DIRECTION intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.RETRACT;

    public double deltaCount = 0; //Need tested value

    public boolean runIntakeMotorToLevelState = false;

    //Constructor`
    public IntakeSlides(HardwareMap hardwareMap){
        intakeMotorLeft = hardwareMap.get(DcMotorEx.class, "intake_motor_left");
        intakeMotorRight = hardwareMap.get(DcMotorEx.class, "intake_motor_right");

        // get a reference to our digitalTouch object.
        intakeTouch = hardwareMap.get(DigitalChannel.class, "intake_reset_ts ");
        // set the digital channel to input.
        intakeTouch.setMode(DigitalChannel.Mode.INPUT);

        intakeDistanceSensor = hardwareMap.get(DistanceSensor.class, "intake_distance");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)intakeDistanceSensor;

        initIntakeSlides();
    }

    //Method is able to initialize the arm
    public void initIntakeSlides(){
        resetIntakeMotorMode();
        intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorLeft.setPositionPIDFCoefficients(5.0);
        intakeMotorRight.setPositionPIDFCoefficients(5.0);
        intakeMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        turnIntakeBrakeModeOn();
        manualResetIntakeMotor();
    }

    //Turns on the brake for Intake motor
    public void turnIntakeBrakeModeOn(){
        intakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Sets outtake slides to Transfer position
    public void moveIntakeSlides(INTAKE_MOTOR_STATE toIntakeMotorState){
        turnIntakeBrakeModeOn();
        intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
        if (intakeMotorCurrentPosition < toIntakeMotorState.motorPosition ) {
            intakeMovementDirection = INTAKE_MOVEMENT_DIRECTION.EXTEND;
        } else {
            intakeMovementDirection = INTAKE_MOVEMENT_DIRECTION.RETRACT;
        }
        intakeMotorLeft.setTargetPosition((int)toIntakeMotorState.motorPosition);
        intakeMotorRight.setTargetPosition((int)toIntakeMotorState.motorPosition);
        intakeMotorState = toIntakeMotorState;
        runIntakeMotorToLevelState = true;
    }

    public void modifyIntakeSlidesLength(double stepSizeFactor, int direction){
        deltaCount = stepSizeFactor * INTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
            intakeMotorNewPosition = (intakeMotorCurrentPosition + direction * deltaCount);
            if (intakeMotorNewPosition < IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition) {
                intakeMotorNewPosition = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition;
                intakeMotorState = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED;
            } else if (intakeMotorNewPosition > IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition) {
                intakeMotorNewPosition = IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition;
                intakeMotorState = IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED;
            } else {
                intakeMotorState = IntakeSlides.INTAKE_MOTOR_STATE.RANDOM;
            }
            intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
            if (intakeMotorCurrentPosition < intakeMotorNewPosition ) {
                intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (intakeMotorNewPosition != intakeMotorCurrentPosition) {
                turnIntakeBrakeModeOn();
                intakeMotorLeft.setTargetPosition((int)intakeMotorNewPosition);
                intakeMotorRight.setTargetPosition((int)intakeMotorNewPosition);
                runIntakeMotorToLevelState = true;
            }
        }
    }

    //sets the Intake motor power
    public void runIntakeMotorToLevel(){
        double power = 0;
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            power = INTAKE_MOTOR_POWER_AUTO;
        } else {
            power = INTAKE_MOTOR_POWER_TELEOP;
        }
        intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runIntakeMotorToLevelState == true){
            intakeMotorLeft.setPower(power);
            intakeMotorRight.setPower(power);
            runIntakeMotorToLevelState = false;
        } else{
            intakeMotorLeft.setPower(0.0);
            intakeMotorRight.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO */
        if (!intakeTouch.getState()) {
            resetIntakeMotorMode();
        }
    }
    public boolean senseIntakeSlidesMinRetracted(){
        return intakeTouch.getState();
    }

    //Resets the arm
    public void resetIntakeMotorMode(){
        DcMotorEx.RunMode runMode = intakeMotorLeft.getMode();
        intakeMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(runMode);
        intakeMotorRight.setMode(runMode);
    }

    public void manualResetIntakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (intakeTouch.getState() && timer.time() < 5000) {
            intakeMotorLeft.setTargetPosition((int) (intakeMotorLeft.getCurrentPosition() - INTAKE_MOTOR_DELTA_COUNT_RESET));
            intakeMotorRight.setTargetPosition((int) (intakeMotorLeft.getCurrentPosition() - INTAKE_MOTOR_DELTA_COUNT_RESET));
            runIntakeMotorToLevelState = true;
            runIntakeMotorToLevel();
        }
        resetIntakeMotorMode();
        turnIntakeBrakeModeOn();
        intakeMotorState = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED;
    }

    public double getDistance(){
        return intakeDistanceSensor.getDistance(DistanceUnit.MM);
    }

    //TODO Protect intake so that it does not hit anything based on distance read by Distance Sensor

}
