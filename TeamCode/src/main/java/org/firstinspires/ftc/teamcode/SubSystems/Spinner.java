package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class Spinner {

    //TODO: Update code as needed for Subsystem1

    public DcMotor spinnerMotor;
    //public CRServo spinnerServo;

    public enum SPINNER_MOTOR_STATE {
        CLOCKWISE,
        ANTICLOCKWISE,
        STOPPED
    }
    public SPINNER_MOTOR_STATE spinnerMotorState = SPINNER_MOTOR_STATE.STOPPED;

    public double spinnerMotorPower = 0.9;//0.6;//0.7;
    public double spinnerMotorPowerAuto = 0.9;//0.6;
    //public SUBSYSTEM1_BUTTON_STATE subsystem1ButtonState;

    public boolean autonomous = false;

    public Spinner (HardwareMap hardwareMap) {
        spinnerMotor = hardwareMap.dcMotor.get("spinner_motor");
        initSpinner();
        //spinnerServo = hardwareMap.crservo.get("spinner_servo");
    }

    public void initSpinner(){
        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
    private void runSpinnerMotor(DcMotor.Direction direction, double power){
        spinnerMotor.setDirection(direction);
        spinnerMotor.setPower(power);
    }
    /**
     * runIntakeMotor checks if the intake is not running and runs the intake
     */
    public void runSpinnerMotorClockwise() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.CLOCKWISE) {
            if (autonomous == false) {
                runSpinnerMotor(DcMotor.Direction.FORWARD, spinnerMotorPower);
            } else {
                runSpinnerMotor(DcMotor.Direction.FORWARD, spinnerMotorPowerAuto);
            }
            spinnerMotorState = SPINNER_MOTOR_STATE.CLOCKWISE;
            //spinnerServo.setPower(1.0);
        }
    }

    /**
     * stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
     * and sets intakeMotorState to INTAKE_MOTOR_STATE.STOPPED
     */
    public void runSpinnerMotorAnticlockwise() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
            if (autonomous == false) {
                runSpinnerMotor(DcMotor.Direction.REVERSE, spinnerMotorPower);
            } else {
                runSpinnerMotor(DcMotor.Direction.REVERSE, spinnerMotorPowerAuto);
            }
            spinnerMotorState = SPINNER_MOTOR_STATE.ANTICLOCKWISE;
            //spinnerServo.setPower(-1.0);
        }
    }

    /**
     * reverseIntakeMotor checks if the intake is not reversing, and sets the intake motor to FORWARD, then also
     * ets intake motor state to REVERSING
     */
    public void stopSpinnerMotor() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.STOPPED) {
            runSpinnerMotor(DcMotor.Direction.FORWARD, 0.0);
            spinnerMotorState = SPINNER_MOTOR_STATE.STOPPED;
            //spinnerServo.setPower(0);
        }

    }
    /**
     * Returns Intake motor state
     */
    public SPINNER_MOTOR_STATE getSpinnerMotorState() {
        return spinnerMotorState;
    }

    public double getSpinnerMotorPower(){
        //return spinnerServo.getPower();
        return spinnerMotor.getPower();
    }
}

