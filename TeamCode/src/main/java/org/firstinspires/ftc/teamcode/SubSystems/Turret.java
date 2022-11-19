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
    public DigitalChannel turretLeftMagneticSensor, turretCenterMagneticSensor, turretRightMagneticSensor;

    public enum TURRET_MOTOR_STATE {
        FACING_FORWARD,
        FACING_BACKWARD_LEFT,
        FACING_BACKWARD_RIGHT,
        FACING_LEFT,
        FACING_RIGHT,
        FACING_RANDOM
    }

    public static TURRET_MOTOR_STATE turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;

    public static final double FACING_FORWARD_POSITION = 0;
    public static final double FACING_BACKWARD_LEFT_POSITION = -1350;
    public static final double FACING_BACKWARD_RIGHT_POSITION = 1350;
    public static final double FACING_LEFT_POSITION = -675;
    public static final double FACING_RIGHT_POSITION = 675;

    public double turretCurrentPosition = FACING_FORWARD_POSITION;
    public double turretNewPosition = FACING_FORWARD_POSITION;

    public static double turretAngleRadians;
    public double turretAngleDegrees;

    public static final double TURRET_POWER = 0.7;

    //value declarations
    public boolean runTurretToLevelState = false;
    public static double TURRET_DELTA_COUNT_MAX = 100; //movement value of turret given clockwise or counterclockwise rotation(changeable)
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
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetTurretMode();
        //set motor direction opposite for rotation
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        turnTurretBrakeModeOn();
        faceForward();
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

    /**
     * Rotate Turret
     * assign to gamepad value once done
     * convert turret position value to degrees
     * @param stepSizeFactor
     */
    public void rotateTurret(double stepSizeFactor){
        turretDeltaCount = (int) stepSizeFactor * TURRET_DELTA_COUNT_MAX;

        if (turretDeltaCount !=0) {
            turretCurrentPosition = turretMotor.getCurrentPosition();
            turretNewPosition = turretCurrentPosition + turretDeltaCount;
            if (turretNewPosition < FACING_BACKWARD_LEFT_POSITION ) {
                turretNewPosition = FACING_BACKWARD_LEFT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_LEFT;
            } else if (turretNewPosition > FACING_BACKWARD_RIGHT_POSITION ) {
                turretNewPosition = FACING_BACKWARD_RIGHT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_RIGHT;
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
        DcMotorEx.RunMode runMode = turretMotor.getMode();
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(runMode);
        runTurretToLevelState = false;
    }

    public void rotateAutonomous(){
        //TODO add elapsed timer
        //uses 2 magnet sensors goes from left one to the center one
        if (turretLeftMagneticSensor.getState() && !turretCenterMagneticSensor.getState()){
            while(!turretCenterMagneticSensor.getState()){
                turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() + 10));
            }
        } else if (turretLeftMagneticSensor.getState() && turretCenterMagneticSensor.getState()){
            turnTurretBrakeModeOff();
            turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        }
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
        // If the Magnetic Limit Swtch is pressed, stop the motor
        /*if (turretCenterMagneticSensor.getState()) {
            turnTurretBrakeModeOff();
            resetTurretMode();
        } else { // Otherwise, run the motor
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() + stepSizeFactor * 50));
            runTurretToPosition(0.2);
        }*/
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        int direction = 1;
        if (directionFactor < 0) {
            direction = -1;
        } else {
            direction = 1;
        }
        while (!turretCenterMagneticSensor.getState() && timer.time() < 2000) {
            turretMotor.setTargetPosition((int) (turretMotor.getCurrentPosition() + direction * 50));
            runTurretToPosition(0.2);
        }
        turnTurretBrakeModeOff();
        resetTurretMode();
        turretMotorState = TURRET_MOTOR_STATE.FACING_FORWARD;
    }
}
