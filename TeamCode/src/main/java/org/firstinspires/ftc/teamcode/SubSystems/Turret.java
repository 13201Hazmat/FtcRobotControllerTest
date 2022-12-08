package org.firstinspires.ftc.teamcode.SubSystems;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * Definition of TURRET Class <BR>
 *
 * Turret consists of one motor which rotates a circular base, of which the arm and shoulder is on<BR>
 *
 * The states are as followed: <BR>
 *     TURRET_MOTOR_STATE - possible positions of the turret, consists of preset positions and random(joystick control)<BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     turnTurretBrakeModeOn, turnTurretBrakeModeOff - sets the turret motor in a state of brake or active <BR>
 *     initTurret, resetTurret - sets the motor and states<BR>
 *     face[x] - moves the turret motor to preset positions of x, e.g. LEFT<BR>
 *     rotateTurret - moves the turret clockwise or counterclockwise based on joystick value,
 *          amount controlled by changing delta value <BR>
 *     runTurretToPosition - applies power to the turret motor, moving it to target positions set by other functions <BR>
 */

public class Turret {


    //TurretMotor declarations
    public DcMotorEx turretMotor;

    //Turret Motor : 5203 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 24mm Length 8mm REX Shaft, 312 RPM, 3.3 - 5V Encoder)
    //Gearing : 1:5
    public static final double TURRET_MOTOR_ENCODER_TICKS = 537.7 * 5;

    public DigitalChannel turretLeftMagneticSensor, turretCenterMagneticSensor, turretRightMagneticSensor;

    public enum TURRET_MOTOR_STATE {
        FACING_FORWARD,
        FACING_BACKWARD_LEFT,
        FACING_BACKWARD_RIGHT,
        MAX_LEFT,
        MAX_RIGHT,
        FACING_LEFT,
        FACING_RIGHT,
        FACING_RANDOM
    }

    public static TURRET_MOTOR_STATE turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;

    public static final double FACING_FORWARD_POSITION = 0;
    public static final double MAX_LEFT_POSITION = (-1350-1350); //-360 deg
    public static final double MAX_RIGHT_POSITION = (1350+1350); //+360 deg
    public static final double FACING_BACKWARD_LEFT_POSITION = -1350; //-180 deg
    public static final double FACING_BACKWARD_RIGHT_POSITION = 1350; //+180 deg
    public static final double FACING_LEFT_POSITION = -675;
    public static final double FACING_RIGHT_POSITION = 675;
    public static final double NINETY_DEGREE_DELTA = 675;

    public static final double FORTY_FIVE_DEGREE_DELTA = NINETY_DEGREE_DELTA/2;

    public double turretCurrentPosition = FACING_FORWARD_POSITION;
    public double turretNewPosition = FACING_FORWARD_POSITION;

    public double turretAngleRadians;
    public double turretAngleDegrees;

    public static final double TURRET_POWER = 0.9;
    public static final double TURRET_RESET_POWER = 0.7;

    //value declarations
    public boolean runTurretToLevelState = false;
    public static double TURRET_DELTA_COUNT_MAX = 300; //movement value of turret given clockwise or counterclockwise rotation(changeable)
    public double turretDeltaCount = 0;

    public Turret(HardwareMap hardwareMap) { //map turretmotor to turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretMotor");
        turretLeftMagneticSensor = hardwareMap.get(DigitalChannel.class, "turretLeftMag");
        turretCenterMagneticSensor = hardwareMap.get(DigitalChannel.class, "turretCenterMag");
        turretRightMagneticSensor = hardwareMap.get(DigitalChannel.class, "turretRightMag");
        turretLeftMagneticSensor.setMode(DigitalChannel.Mode.INPUT);
        turretCenterMagneticSensor.setMode(DigitalChannel.Mode.INPUT);
        turretLeftMagneticSensor.setMode(DigitalChannel.Mode.INPUT);
        initTurret();
    }

    //turret initialization
    public void initTurret(){
        resetTurretMode();
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        turretMotor.setPositionPIDFCoefficients(5.0);
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turnTurretBrakeModeOn();
        //manualResetTurret(1);
    }

    //Turns on the brake for arm motor
    public void turnTurretBrakeModeOn(){
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns brake for arm motor off
    public void turnTurretBrakeModeOff() {
        turretMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void faceForward() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;
        turretMotor.setTargetPosition((int)FACING_FORWARD_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }
    //commented out, as use not needed yet

    public void faceBackward(){
        turretCurrentPosition = turretMotor.getCurrentPosition();
        if (turretCurrentPosition <0) {
            turretMotor.setTargetPosition((int)FACING_BACKWARD_LEFT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_LEFT;
        } else {
            turretMotor.setTargetPosition((int)FACING_BACKWARD_RIGHT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_RIGHT;
        }
        runTurretToLevelState = true;
    }

    public void faceLeft() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_LEFT;
        turretMotor.setTargetPosition((int)FACING_LEFT_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }
    public void faceRight() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_RIGHT;
        turretMotor.setTargetPosition((int)FACING_RIGHT_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }

    public void moveTurretToAngle(double turretAnglePosition){
        turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
        turretMotor.setTargetPosition((int)turretAnglePosition);
        runTurretToLevelState = true;
    }

    public void moveTurretPlus90(){
        if ((turretMotor.getCurrentPosition() + NINETY_DEGREE_DELTA) < MAX_RIGHT_POSITION ) {
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() + NINETY_DEGREE_DELTA));
            turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
        } else {
            turretMotor.setTargetPosition((int)MAX_RIGHT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.MAX_RIGHT;
        }

        runTurretToLevelState = true;
    }
    public void moveTurretMinus90(){
        if ((turretMotor.getCurrentPosition() - NINETY_DEGREE_DELTA) > MAX_LEFT_POSITION ) {
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() - NINETY_DEGREE_DELTA));
            turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
        } else {
            turretMotor.setTargetPosition((int)MAX_LEFT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.MAX_LEFT;
        }
        runTurretToLevelState = true;
    }

    public void moveTurretMinus45(){
        if ((turretMotor.getCurrentPosition() - NINETY_DEGREE_DELTA) > MAX_LEFT_POSITION ) {
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() - NINETY_DEGREE_DELTA/2));
            turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
        } else {
            turretMotor.setTargetPosition((int)MAX_LEFT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.MAX_LEFT;
        }
        runTurretToLevelState = true;
    }

    /**
     * Rotate Turret
     * assign to gamepad value once done
     * convert turret position value to degrees
     * @param stepSizeFactor
     */
    public void rotateTurret(double stepSizeFactor){
        turretDeltaCount = (int) (stepSizeFactor * TURRET_DELTA_COUNT_MAX);

        if (turretDeltaCount !=0) {
            turretCurrentPosition = turretMotor.getCurrentPosition();
            turretNewPosition = turretCurrentPosition + turretDeltaCount;
            if (turretNewPosition < MAX_LEFT_POSITION ) {
                turretNewPosition = MAX_LEFT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.MAX_LEFT;
            } else if (turretNewPosition > MAX_RIGHT_POSITION ) {
                turretNewPosition = MAX_RIGHT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.MAX_RIGHT;
            } else {
                turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
            }
            if (turretNewPosition != turretCurrentPosition) {
                turretMotor.setTargetPosition((int)turretNewPosition);
                runTurretToLevelState = true;
            }
        }
    }

    public void calculateTurretAngle(){
        turretAngleDegrees = 180/(FACING_RIGHT_POSITION) * turretMotor.getCurrentPosition();
        turretAngleRadians = Math.toRadians(turretAngleDegrees);
        SystemState.TurretAngleRadians = turretAngleRadians;
    }

    public void resetTurretMode() {
        //DcMotorEx.RunMode runMode = turretMotor.getMode();
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //turretMotor.setMode(runMode);
        runTurretToLevelState = false;
    }

    public void rotateAutonomousMagSensor(){
        //TODO add elapsed timer
        //uses 2 magnet sensors goes from left one to the center one
        while(!turretCenterMagneticSensor.getState()){
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() - 20));
            runTurretToLevelState = true;
            runTurretToPosition(TURRET_RESET_POWER);
        }
        resetTurretMode();
        turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;
    }


    public void rotateAutoInitTurnAndReset(){
        //TODO add elapsed timer
        //uses 2 magnet sensors goes from left one to the center one
        turretMotor.setTargetPosition((int) (/*turretMotor.getCurrentPosition()*/ - FORTY_FIVE_DEGREE_DELTA));
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_RESET_POWER);
        resetTurretMode();
    }

    public void rotateAutoTurnToAngle(double turretAnglePosition) {
        turretMotor.setTargetPosition((int) turretAnglePosition);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        turretMotor.setPower(TURRET_POWER);
    }


    public void runTurretToPosition(double power){//receive value from testing
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runTurretToLevelState == true) {
            turretMotor.setPower(power);
            runTurretToLevelState = false;
        } else {
            turretMotor.setPower(0.0);
        }
    }

    public void manualResetTurret(double directionFactor){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        int direction = 1;
        if (directionFactor < 0) {
            direction = -1;
        } else {
            direction = 1;
        }
        while (turretCenterMagneticSensor.getState() && timer.time() < 7000) {
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() + direction * 20));
            runTurretToLevelState = true;
            runTurretToPosition(TURRET_RESET_POWER);
        }
        turnTurretBrakeModeOff();
        resetTurretMode();
        turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;
    }
}
