package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    //Initialization of intakemotor
    public DcMotorEx intakeMotor = null;
    public Servo intakeLiftServo;

    public enum INTAKE_MOTOR_STATE{
        INTAKE_MOTOR_RUNNING,
        INTAKE_MOTOR_STOPPED,
        INTAKE_MOTOR_REVERSING
    }
    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;
    public INTAKE_MOTOR_STATE intakeMotorPrevState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;

    public enum INTAKE_ROLLER_HEIGHT{
        INTAKE_ROLLER_LIFTED(0), //UPDATE FOR THIS YEAR
        INTAKE_ROLLER_DROPPED(0);

        private double liftPosition;

        INTAKE_ROLLER_HEIGHT(double moveLeftPosition){
            this.liftPosition = moveLeftPosition;
        }
        public double getLiftPosition(){
            return liftPosition;
        }
    }



    public INTAKE_ROLLER_HEIGHT intakeRollerHeightState = INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED;

    public double intakeMotorPower = 1.0;

    public Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeLiftServo = hardwareMap.get(Servo.class, "intake_lift_servo");
        initIntake();
    }

    public void initIntake(){
        intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;
        intakeMotor.setPower(0);
        moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED);
    }

    public void moveRollerHeight(INTAKE_ROLLER_HEIGHT intakeRollerHeight){
        intakeLiftServo.setPosition(intakeRollerHeight.liftPosition);
        intakeRollerHeightState = intakeRollerHeight;
    }

    public void toggleRollerHeight(){
        if (intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED);
        } else {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED);
        }
    }

    public void startIntakeInward(){
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING){
            runIntakeMotor(DcMotor.Direction.FORWARD, intakeMotorPower);
            intakeMotorPrevState = intakeMotorState;
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_RUNNING;
        }
    }

    public void reverseIntake() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING) {
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING;
        }
    }

    public void stopIntakeMotor() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED) {
            runIntakeMotor(DcMotorSimple.Direction.FORWARD, 0.0);
            intakeMotorPrevState = intakeMotorState;
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_STOPPED;
        }
    }

    public void runIntakeMotor(DcMotor.Direction direction, double intakePower) {
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(intakePower);
    }

    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Intake");
        telemetry.addData("    State", getIntakeState());
        telemetry.addData("    Roller Height State", intakeRollerHeightState);
        telemetry.addData("    Roller Servo Position", intakeLiftServo.getPosition());
        telemetry.addLine("=============");
    }

}