package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Spinner consists of system provided spinner controls and adds functionality to the selection made on spinner. <BR>
 *
 * The states are as followed: <BR>
 *     SPINNER_MOTOR_STATE_CLOCKWISE for one state - This is when spinner is rotating
 *     Clockwise for a specific alliance side <BR>
 *     SPINNER_MOTOR_STATE_ANTICLOCKWISE for another state - This is when spinner is
 *     rotating Anticlockwise for a specific alliance side <BR>
 *     SPINNER_MOTOR_STATE_STOPPED for another state - This is when spinner is not rotating
 *     at all  <BR>
 *
 * The functions are as followed: Example assumes a motor like an spinner <BR>
 *     initSpinner checks if the spinner is ready to start  <BR>
 *     runSpinnerMotor initializes the power and direction of the motor <BR>
 *     runSpinnerMotorClockwise checks if the spinner is already not clockwise
 *     and spins the motor clockwise <BR>
 *     runSpinnerMotorAnticlockwise checks if the spinner is already not anticlockwise and
 *     spins the motor anticlockwise <BR>
 *     stopSpinnerMotor checks if spinner is not stopped, then stops the spinner
 *     motor <BR>
 */
public class Spinner {

    //TODO: Update code as needed for Subsystem1

    public DcMotor spinnerMotor;

    public enum SPINNER_MOTOR_STATE {
        CLOCKWISE,
        ANTICLOCKWISE,
        STOPPED
    }
    public SPINNER_MOTOR_STATE spinnerMotorState = SPINNER_MOTOR_STATE.STOPPED;

    public double spinnerMotorPower = 0.5;
    //public SUBSYSTEM1_BUTTON_STATE subsystem1ButtonState;

    /**
     * Parameter that register all hardware devices for Spinner Subsystem
     * @param hardwareMap
     */
    public Spinner (HardwareMap hardwareMap) {
        spinnerMotor = hardwareMap.dcMotor.get("spinner_motor");
        initSpinner();
    }

    /**
     * Starts the Spinner Motor
     */
    public void initSpinner(){
        spinnerMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    /**
     * Initializes the Parameter of power and direction of the motor
     * @param direction
     * @param power
     */
    private void runSpinnerMotor(DcMotor.Direction direction, double power){
        spinnerMotor.setDirection(direction);
        spinnerMotor.setPower(power);
    }
    /**
     * runSpinnerMotorClockwise checks if the spinner is not clockwise and
     * runs the spinner clockwise
     */
    public void runSpinnerMotorClockwise() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.CLOCKWISE) {
            runSpinnerMotor(DcMotor.Direction.FORWARD, spinnerMotorPower);
            spinnerMotorState = SPINNER_MOTOR_STATE.CLOCKWISE;
        }
    }

    /**
     * runSpinnerMotorAnticlockwise checks if the spinner has not spun Anticlockwise, it sets
     * the spinner power to and sets spinnerMotorState to SPINNER_MOTOR_STATE.ANTICLOCKWISE
     */
    public void runSpinnerMotorAnticlockwise() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.ANTICLOCKWISE) {
            runSpinnerMotor(DcMotor.Direction.REVERSE, spinnerMotorPower);
            spinnerMotorState = SPINNER_MOTOR_STATE.ANTICLOCKWISE;
       }
    }

    /**
     * stopSpinnerMotor checks if the spinner is not stopped, and sets the spinner
     * motor to STOPPED
     */
    public void stopSpinnerMotor() {
        if(spinnerMotorState != SPINNER_MOTOR_STATE.STOPPED) {
                runSpinnerMotor(DcMotor.Direction.FORWARD, 0.0);
                spinnerMotorState = SPINNER_MOTOR_STATE.STOPPED;
        }
    }
    /**
     * Returns Intake motor state
     */
    public SPINNER_MOTOR_STATE getSpinnerMotorState() {
        return spinnerMotorState;
    }

    public double getSpinnerMotorPower(){
        return spinnerMotor.getPower();
    }
}

