package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

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
    public DcMotorEx outtakeMotorLeft, outtakeMotorRight;

    //Outtake Motor : 5203 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 8mm REX Shaft, 435 RPM, 3.3 - 5V Encoder)
    public static final double OUTTAKE_MOTOR_ENCODER_TICKS = 384.5;

    public DigitalChannel outtakeTouch;  // Hardware Device Object

    public Servo outtakeTurretServo;

    //Outtake Motor states
    public enum OUTTAKE_MOTOR_STATE {
        MIN_RETRACTED (0), //Position
        TRANSFER (0),
        LOW_JUNCTION (0),
        MEDIUM_JUNCTION (0),
        HIGH_JUNCTION (0),
        MAX_EXTENDED(0),
        RANDOM(0);

        private final double motorPosition;
        OUTTAKE_MOTOR_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public OUTTAKE_MOTOR_STATE outtakeMotorState = OUTTAKE_MOTOR_STATE.MIN_RETRACTED;

    public double outtakeMotorCurrentPosition = outtakeMotorState.motorPosition;
    public double outtakeMotorNewPosition = outtakeMotorState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 200;//200;//200 //need tested values
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
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_motor_left");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_motor_right");

        // get a reference to our digitalTouch object.
        outtakeTouch = hardwareMap.get(DigitalChannel.class, "outtake_reset_ts ");

        // set the digital channel to input.
        outtakeTouch.setMode(DigitalChannel.Mode.INPUT);
        initOuttakeSlides();
    }

    //Method is able to initialize the arm
    public void initOuttakeSlides(){
        resetOuttakeMotorMode();
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorLeft.setPositionPIDFCoefficients(5.0);
        outtakeMotorRight.setPositionPIDFCoefficients(5.0);
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        turnOuttakeBrakeModeOn();
        manualResetOuttakeMotor();
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }



    public void modifyOuttakeSlidesLength(double stepSizeFactor){
        deltaCount = stepSizeFactor * OUTTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
            outtakeMotorNewPosition = (outtakeMotorCurrentPosition + deltaCount);
            if (outtakeMotorNewPosition < OUTTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition;
                outtakeMotorState = OUTTAKE_MOTOR_STATE.MIN_RETRACTED;
            } else if (outtakeMotorNewPosition > OUTTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition;
                outtakeMotorState = OUTTAKE_MOTOR_STATE.MAX_EXTENDED;
            } else {
                outtakeMotorState = OUTTAKE_MOTOR_STATE.RANDOM;
            }
            outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
            if (outtakeMotorCurrentPosition < outtakeMotorNewPosition ) {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (outtakeMotorNewPosition != outtakeMotorCurrentPosition) {
                turnOuttakeBrakeModeOn();
                outtakeMotorLeft.setTargetPosition((int)outtakeMotorNewPosition);
                outtakeMotorRight.setTargetPosition((int)outtakeMotorNewPosition);
                runOuttakeMotorToLevelState = true;
            }
        }
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            power = OUTTAKE_MOTOR_POWER_AUTO;
        } else {
            power = OUTTAKE_MOTOR_POWER_TELEOP;
        }
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runOuttakeMotorToLevelState == true){
            outtakeMotorLeft.setPower(power);
            outtakeMotorRight.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            outtakeMotorLeft.setPower(0.0);
            outtakeMotorRight.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO */
        if (!outtakeTouch.getState()) {
            resetOuttakeMotorMode();
        }
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode = outtakeMotorLeft.getMode();
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorLeft.setMode(runMode);
        outtakeMotorRight.setMode(runMode);
    }

    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (outtakeTouch.getState() && timer.time() < 5000) {
            outtakeMotorLeft.setTargetPosition((int) (outtakeMotorLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
            outtakeMotorRight.setTargetPosition((int) (outtakeMotorLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
            runOuttakeMotorToLevelState = true;
            runOuttakeMotorToLevel();
        }
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOn();
        outtakeMotorState = OUTTAKE_MOTOR_STATE.MIN_RETRACTED;
    }
    ;


    public enum TURRET_STATE{
        MAX_LEFT (0.7),
        INIT (0.5),
        MAX_RIGHT (0.3),
        RANDOM (0.5);
        private final double turretPosition;
        private TURRET_STATE(double turretPosition){
            this.turretPosition = turretPosition;
        }
    }

    public TURRET_STATE turretState = TURRET_STATE.INIT;

    public double TURRET_DELTA = 0.01;
    public double TURRET_TURBO_DELTA = 0.03;

    public void initTurret(){
        outtakeTurretServo.setPosition(TURRET_STATE.INIT.turretPosition);
        turretState = TURRET_STATE.INIT;
    }

    public void moveTurret(TURRET_STATE toTurretState){
        outtakeTurretServo.setPosition(toTurretState.turretPosition);
        turretState = toTurretState;
    }

    public void moveTurretDelta(){
        //TODO
    }

}





