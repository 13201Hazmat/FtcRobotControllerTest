package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
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

    public enum TURRET_MOTOR_STATE {
        FACING_FORWARD,
        FACING_BACKWARD_LEFT,
        FACING_BACKWARD_RIGHT,
        FACING_LEFT,
        FACING_RIGHT,
        FACING_RANDOM
    }

    public static TURRET_MOTOR_STATE turretMotorState;

    public static final int FACING_FORWARD_POSITION = 0;
    public static final int FACING_BACKWARD_LEFT_POSITION = -1350;
    public static final int FACING_BACKWARD_RIGHT_POSITION = 1350;
    public static final int FACING_LEFT_POSITION = -675;
    public static final int FACING_RIGHT_POSITION = 675;

    public int turretCurrentPosition = FACING_FORWARD_POSITION;
    public int turretNewPosition = FACING_FORWARD_POSITION;

    public static double turretAngleRadians;
    public double turretAngleDegrees;

    public static final double TURRET_POWER = 0.7;

    //value declarations
    public boolean runTurretToLevelState = false;
    public static int TURRET_DELTA_COUNT_MAX = 100; //movement value of turret given clockwise or counterclockwise rotation(changeable)
    public int turretDeltaCount = 0;

    public Turret(HardwareMap hardwareMap) { //map turretmotor to turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        initTurret();
    }

    //turret initialization
    public void initTurret(){
        turretMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        resetTurret();
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
        turretMotor.setTargetPosition(FACING_FORWARD_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }
    //commented out, as use not needed yet

    public void faceBackward(){
        turretCurrentPosition = turretMotor.getCurrentPosition();
        if (turretCurrentPosition <0) {
            turretMotor.setTargetPosition(FACING_BACKWARD_LEFT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_LEFT;
        } else {
            turretMotor.setTargetPosition(FACING_BACKWARD_RIGHT_POSITION);
            turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_RIGHT;
        }
        runTurretToLevelState = true;
    }

    public void faceLeft() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_LEFT;
        turretMotor.setTargetPosition(FACING_LEFT_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }
    public void faceRight() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_RIGHT;
        turretMotor.setTargetPosition(FACING_RIGHT_POSITION);
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
                turretMotor.setTargetPosition(turretNewPosition);
                runTurretToLevelState = true;
            }
        }
    }

    public void calculateTurretAngle(){
        turretAngleDegrees = 180/(FACING_RIGHT_POSITION) * turretMotor.getCurrentPosition();
        turretAngleRadians = Math.toRadians(turretAngleDegrees);
        SystemState.TurretAngleRadians = turretAngleRadians;
    }

    public void resetTurret(){
        DcMotorEx.RunMode runMode = turretMotor.getMode();
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(runMode);
        runTurretToLevelState = false;

    }
    public void runTurretToPosition(double power){//receive value from testing
        turretMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runTurretToLevelState == true){
            turretMotor.setPower(power);
            runTurretToLevelState = false;
        } else{
            turretMotor.setPower(0.0);
        }
    }

}
