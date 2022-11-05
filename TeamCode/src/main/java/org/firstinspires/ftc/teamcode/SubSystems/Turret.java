package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
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
    public static final int FACING_BACKWARD_LEFT_POSITION = -537*7+300;
    public static final int FACING_BACKWARD_RIGHT_POSITION = 537*7-300;
    public static final int FACING_LEFT_POSITION = -537*7/2;
    public static final int FACING_RIGHT_POSITION = 537*7/2;

    public int turretMotorPosition;

    public double turretPower = 0.5;

    //value declarations
    public boolean runTurretToLevelState = false;
    public static int TURRET_DELTA_COUNT_MAX = 50; //movement value of turret given clockwise or counterclockwise rotation(changeable)
    public int turretDeltaCount = 0;

    public Turret(HardwareMap hardwareMap) { //map turretmotor to turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        initTurret();
    }

    //turret initialization
    public void initTurret(){
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

    public void faceBackwardLeft() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_LEFT;
        turretMotor.setTargetPosition(FACING_BACKWARD_LEFT_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
    }

    public void faceBackwardRight() {
        turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_RIGHT;
        turretMotor.setTargetPosition(FACING_BACKWARD_RIGHT_POSITION);
        runTurretToLevelState = true;
        //assign value after testing
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
     * @param joyStickValue
     */
    public void rotateTurret(double joyStickValue){
        turretMotorPosition = turretMotor.getCurrentPosition();
        if (joyStickValue > 0.2) {
            turretDeltaCount = (int) (Math.pow((joyStickValue * 1.25 - 0.25), 3) * TURRET_DELTA_COUNT_MAX);
        } else if (joyStickValue < -0.2) {
            turretDeltaCount = (int) (Math.pow((joyStickValue * 1.25 + 0.25), 3) * TURRET_DELTA_COUNT_MAX);
        } else {
            turretDeltaCount = 0;
        }
        if ((turretDeltaCount !=0)
                && (turretMotorPosition >=  FACING_BACKWARD_LEFT_POSITION )
                && (turretMotorPosition <= FACING_BACKWARD_RIGHT_POSITION)){
            turnTurretBrakeModeOn();
            turretMotorPosition = (int) (turretMotorPosition + joyStickValue * TURRET_DELTA_COUNT_MAX);
            if (turretMotorPosition < FACING_BACKWARD_LEFT_POSITION ) {
                turretMotorPosition = FACING_BACKWARD_LEFT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_LEFT;
            } else if (turretMotorPosition > FACING_BACKWARD_RIGHT_POSITION ) {
                turretMotorPosition = FACING_BACKWARD_RIGHT_POSITION;
                turretMotorState = TURRET_MOTOR_STATE.FACING_BACKWARD_RIGHT;
            } else {
                turretMotorState = TURRET_MOTOR_STATE.FACING_RANDOM;
            }
            turretMotor.setTargetPosition(turretMotorPosition);
            runTurretToLevelState = true;
        }
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
