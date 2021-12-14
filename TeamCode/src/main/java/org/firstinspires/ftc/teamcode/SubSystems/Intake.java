package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
//import com.qualcomm.robotcore.hardware.Servo;

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
public class Intake {

    public DcMotor intakeMotor = null;


    public enum INTAKE_MOTOR_STATE {
        RUNNING,
        STOPPED,
        REVERSING
    }

    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;

    public double intakeMotorPower1 = 1.0;//0.9;
    public double intakeMotorPower2 = 0.8;

    /*public enum INTAKE_BUTTON_STATE {
        ON,
        OFF
    }*/
    //public INTAKE_BUTTON_STATE intakeButtonState;

    /**
     *Parameter that register all hardware devices for Intake Subsystem
     * @param hardwareMap
     */
    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.dcMotor.get("intake_motor");
        initIntake();
    }

    public void initIntake(){

    }

    /**
     * Starts the intake to running if it is not running before
     */
    public void startIntakeMotorInward() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.RUNNING) {
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower2);
            intakeMotorState = INTAKE_MOTOR_STATE.RUNNING;
        }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * sets intake motor state to REVERSING
     */
    public void startIntakeMotorOutward() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            runIntakeMotor(DcMotor.Direction.FORWARD, intakeMotorPower1);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.STOPPED) {
            runIntakeMotor(DcMotor.Direction.REVERSE, 0.0);
            intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
       }
    }

    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    private void runIntakeMotor(DcMotor.Direction direction, double power){
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(power);
    }


    /**
     * Returns Intake motor state
     */
    public INTAKE_MOTOR_STATE getIntakeMotorState() {
        return intakeMotorState;
    }

    public double getIntakeMotorPower(){
    return intakeMotor.getPower();}
}