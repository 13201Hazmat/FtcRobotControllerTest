package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

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
        INTAKE_ROLLER_INIT_AUTO(0.55,7),
        INTAKE_ROLLER_HEIGHT_ABOVE_STACK(0.18,6),
        INTAKE_ROLLER_LIFTED_5(0.145,5),
        INTAKE_ROLLER_LIFTED_4(0.11,4),
        INTAKE_ROLLER_LIFTED_3(0.08,3),
        INTAKE_ROLLER_LIFTED_2(0.05,2),
        INTAKE_ROLLER_DROPPED(0,1);

        private double liftPosition;
        private int index;

        INTAKE_ROLLER_HEIGHT(double liftPosition, int index){
            this.liftPosition = liftPosition;
            this.index = index;
        }
        public double getLiftPosition(){
            return liftPosition;
        }
        public int getIndex() {return index;}

        public INTAKE_ROLLER_HEIGHT byIndex(int ord) {
            if (ord <1) ord = 1;
            if (ord >6) ord = 6;
            for (INTAKE_ROLLER_HEIGHT a : INTAKE_ROLLER_HEIGHT.values()) {
                if (a.index == ord) {
                    return a;
                }
            }
            return null;
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
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_INIT_AUTO);
        }
    }

    public void moveRollerHeight(INTAKE_ROLLER_HEIGHT targetIntakeRollerHeight){
        intakeLiftServo.setPosition(targetIntakeRollerHeight.liftPosition);
        intakeRollerHeightState = targetIntakeRollerHeight;
    }

    public void toggleRollerHeight(){
        if (intakeRollerHeightState != INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
        } else {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED);
        }
    }

    public void moveIntakeRollerOnePixelDown(){
        if (intakeRollerHeightState != INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED) {
            moveRollerHeight(intakeRollerHeightState.byIndex((intakeRollerHeightState.getIndex()-1)));
        }
    }

    public void moveIntakeRollerToLevel(int level){
        if (level > 5) {
            level = 5;
        }
        if (level < 1) {
            level = 1;
        }
        moveRollerHeight(intakeRollerHeightState.byIndex(level));
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
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_5);
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
        telemetry.addData("    Motor Power", intakeMotorPower);
        telemetry.addLine("=============");
    }

}