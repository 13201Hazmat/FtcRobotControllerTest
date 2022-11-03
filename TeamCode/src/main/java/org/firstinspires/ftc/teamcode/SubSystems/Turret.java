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

    //value declarations
    public boolean runTurretToLevelState = false;
    public static int TURRET_DELTA_COUNT = (int) Math.toDegrees(5); //movement value of turret given clockwise or counterclockwise rotation(changeable)
    public static double turretMotorPosition = (int) Math.toDegrees(0); //change to degrees, degree position of turret


    // Used for both TeleOp and Autonomous
    public TURRET_STATE turret_state;// declare turret state enum

    public enum TURRET_STATE{
        FACING_FORWARD, //get values from test robot
        FACING_BACKWARD, //get values from test robot
        FACING_LEFT, //get values from test robot
        FACING_RIGHT //get values from test robot
    }

    public Turret(HardwareMap hardwareMap) { //map turretmotor to turret
        turretMotor = hardwareMap.get(DcMotorEx.class, "turretmotor");
        initTurret();
    }

    public void faceForward() {
        turret_state = TURRET_STATE.FACING_FORWARD;
        runTurretToLevelState = true;
        //assign value after testing
    }
    //commented out, as use not needed yet

    public void faceBackward() {
        turret_state = TURRET_STATE.FACING_BACKWARD;
        runTurretToLevelState = true;
        //assign value after testing
    }
    public void faceLeft() {
        turret_state = TURRET_STATE.FACING_LEFT;
        runTurretToLevelState = true;
        //assign value after testing
    }
    public void faceRight() {
        turret_state = TURRET_STATE.FACING_RIGHT;
        runTurretToLevelState = true;
        //assign value after testing
    }

    //turret initialization
    public void initTurret(){
        resetTurret();
        //set motor direction opposite for rotation
        turretMotor.setDirection(DcMotorEx.Direction.FORWARD);
        faceForward();
    }


    /**
     * Move Turret Left
     * convert turret position value to degrees
     * assign to gamepad value once done
     * @param v
     */
    public void moveTurretCounterClockwise(double v){
        faceLeft();
        if (turretMotorPosition > -180 && turretMotorPosition <= 0){
            turretMotorPosition = turretMotorPosition - TURRET_DELTA_COUNT;
        }
        runTurretToLevelState = true;
    }
    /**
     * Move Turret Right
     * assign to gamepad value once done
     * convert turret position value to degrees
     * @param v
     */
    public void moveTurretClockwise(double v){
        faceRight();
        if (turretMotorPosition < 180 && turretMotorPosition <= 0){
            turretMotorPosition = turretMotorPosition + TURRET_DELTA_COUNT;
        }
        runTurretToLevelState = true;
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
