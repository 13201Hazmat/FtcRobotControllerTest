package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

/**
 * Definition of Outtake Slides Class <BR>
 *
 * Arm consists of linear slides starting from the shoulder(pivot) and can extend to a fixed length
 * by the driver, holding the hand at the end<BR>
 *
 * The states are as followed: <BR>
 *     ARM_STATE - example if linear slides are either fully extended or fully retracted  <BR>
 *     ARM_MOTOR_POSITION -  linear slides are at preset positions <BR>
 *
 * The functions are as followed: <BR>
 *     initArm resets the motors and positions <BR>
 *     turnArmBrakeModeOn and turnArmBrakeModeOff puts the arm motor in a stopped or active state <BR>
 *     moveToJunction functions set the target position to preset positions corresponding to function<BR>
 *     extendArm and retractArm functions extend and retract arm based on a delta value determined <BR>
 *     by the joystick <BR>
 *     resetArm resets the arm to the original position and states <BR>
 *     runArmToLevel runs the arm to the levels determined by the other functions <BR>
 */

public class OuttakeSlides {
    //Initialization of outtakemotor
    public DcMotorEx outtakeMotor;

    //Outtake Motor : 5203 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 8mm REX Shaft, 435 RPM, 3.3 - 5V Encoder)
    public static final double OUTTAKE_MOTOR_ENCODER_TICKS = 145.6;//384.5;

    public DigitalChannel outtakeTouch;  // Hardware Device Object

    public Servo outtakeTurretServo;

    //Outtake Motor states
    public enum OUTTAKE_SLIDE_STATE {
        MIN_RETRACTED (0), //Position
        TRANSFER (100),
        LOW_JUNCTION (200),
        MEDIUM_JUNCTION (450),
        HIGH_JUNCTION (640),
        MAX_EXTENDED(660),
        RANDOM(0);

        private final double motorPosition;
        OUTTAKE_SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public OUTTAKE_SLIDE_STATE outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;

    public double outtakeMotorCurrentPosition = outtakeSlidesState.motorPosition;
    public double outtakeMotorNewPosition = outtakeSlidesState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 100;//200;//200 //need tested values
    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;

    //Different constants of arm speed
    public static final double OUTTAKE_MOTOR_POWER_TELEOP = 0.8;
    public static final double OUTTAKE_MOTOR_POWER_AUTO = 0.8;
    public enum OUTTAKE_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }
    public OUTTAKE_MOVEMENT_DIRECTION outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;

    public double deltaCount = 0; //Need tested value

    public boolean runOuttakeMotorToLevelState = false;

    //Constructor`
    public OuttakeSlides(HardwareMap hardwareMap){

        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtake_motor");

        // get a reference to our digitalTouch object.
        outtakeTouch = hardwareMap.get(DigitalChannel.class, "outtake_reset_ts ");

        // set the digital channel to input.
        outtakeTouch.setMode(DigitalChannel.Mode.INPUT);

        //Turret
        outtakeTurretServo = hardwareMap.get(Servo.class, "outtake_turret");

        initOuttakeSlides();
    }

    //Method is able to initialize the arm
    public void initOuttakeSlides(){
        resetOuttakeMotorMode();
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setPositionPIDFCoefficients(5.0);
        outtakeMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turnOuttakeBrakeModeOff();
        manualResetOuttakeMotor();
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(OUTTAKE_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeMotorCurrentPosition = outtakeMotor.getCurrentPosition();
        /*if (outtakeMotorCurrentPosition < toOuttakeMotorState.motorPosition ) {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
        } else {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
        }

         */
        outtakeMotor.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void modifyOuttakeSlidesLength(double stepSizeFactor){
        deltaCount = stepSizeFactor * OUTTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            outtakeMotorCurrentPosition = outtakeMotor.getCurrentPosition();
            outtakeMotorNewPosition = (outtakeMotorCurrentPosition + deltaCount);
            if (outtakeMotorNewPosition < OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition;
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
            } else if (outtakeMotorNewPosition > OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition;
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.MAX_EXTENDED;
            } else {
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.RANDOM;
            }
            outtakeMotorCurrentPosition = outtakeMotor.getCurrentPosition();
            if (outtakeMotorCurrentPosition < outtakeMotorNewPosition ) {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (outtakeMotorNewPosition != outtakeMotorCurrentPosition) {
                turnOuttakeBrakeModeOn();
                outtakeMotor.setTargetPosition((int)outtakeMotorNewPosition);
                runOuttakeMotorToLevelState = true;
            }
        }
    }

    /*
    public void modifyOuttakeSlidesLength(double power){
        //outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnOuttakeBrakeModeOn();
        double outtaMotorCurrentPosition = outtakeMotor.getCurrentPosition();
        if ((power > 0.01 && outtaMotorCurrentPosition < OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition) ||
                (power < 0.01 && outtaMotorCurrentPosition > OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition )) {
            outtakeMotor.setPower(power);
        } else {
            outtakeMotor.setPower(0);
        }
    }

     */


    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (outtakeSlidesState == OUTTAKE_SLIDE_STATE.MIN_RETRACTED) {
            turnOuttakeBrakeModeOff();
        } else {
            turnOuttakeBrakeModeOn();
        }
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            power = OUTTAKE_MOTOR_POWER_AUTO;
        } else {
            power = OUTTAKE_MOTOR_POWER_TELEOP;
        }
        outtakeMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runOuttakeMotorToLevelState == true){
            outtakeMotor.setPower(power);
            if (!outtakeMotor.isBusy()) runOuttakeMotorToLevelState = false;
        } else{
            outtakeMotor.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO
        if (!outtakeTouch.getState()) {
            resetOuttakeMotorMode();
        }
         */
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode = outtakeMotor.getMode();
        outtakeMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(runMode);
    }

    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (outtakeTouch.getState() && timer.time() < 5000) {
            outtakeMotor.setTargetPosition((int) (outtakeMotor.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
            runOuttakeMotorToLevelState = true;
            runOuttakeMotorToLevel();
        }
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
    }

    public enum TURRET_STATE{
        MAX_LEFT (0.34),
        CENTER(0.44),
        MAX_RIGHT (0.54),
        INIT(0.1),
        RANDOM (0.5),
        AUTO_LEFT(0.39),
        AUTO_RIGHT(0.49);
        private final double turretPosition;
        private TURRET_STATE(double turretPosition){
            this.turretPosition = turretPosition;
        }
    }

    public TURRET_STATE turretState = TURRET_STATE.CENTER;

    public double TURRET_DELTA = 0.01;
    public double TURRET_TURBO_DELTA = 0.09;

    public void initTurret(){
        outtakeTurretServo.setPosition(TURRET_STATE.CENTER.turretPosition);
        turretState = TURRET_STATE.INIT;
    }

    public void moveTurret(TURRET_STATE toTurretState){
        outtakeTurretServo.setPosition(toTurretState.turretPosition);
        turretState = toTurretState;
    }

    public void moveTurretDelta(double stepSizeFactor){
        double deltaTurret = outtakeTurretServo.getPosition() + (stepSizeFactor /90 );
        if(deltaTurret > TURRET_STATE.MAX_RIGHT.turretPosition){
            deltaTurret = TURRET_STATE.MAX_RIGHT.turretPosition;
        }
        if(deltaTurret<TURRET_STATE.MAX_LEFT.turretPosition){
            deltaTurret = TURRET_STATE.MAX_LEFT.turretPosition;
        }
        outtakeTurretServo.setPosition(deltaTurret);
        turretState = TURRET_STATE.RANDOM;
    }
    public boolean isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE outttakeSlideState) {
        return ((outtakeMotor.getCurrentPosition() < outttakeSlideState.motorPosition + 10) &&
                ((outtakeMotor).getCurrentPosition()  > outttakeSlideState.motorPosition - 10));
    }

}





