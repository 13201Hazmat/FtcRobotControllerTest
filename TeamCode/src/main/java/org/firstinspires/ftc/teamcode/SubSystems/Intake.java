package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake {

    //Initialization of intakemotor
    public DcMotorEx intakeMotor = null;
    public Servo stackIntakeLiftServo;
    public CRServo stackIntakeServoLeft, stakeIntakeServoRight;

    public boolean stackIntakeActivated = false;
    boolean reverseStackIntakeFlag = false;

    public ElapsedTime stackIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime reverseStackIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public enum STACK_INTAKE_SERVO_STATE {
        COLLECT,
        REVERSE,
        STOPPED
    }
    public STACK_INTAKE_SERVO_STATE stackIntakeServoState = STACK_INTAKE_SERVO_STATE.STOPPED;

    public enum INTAKE_MOTOR_STATE{
        RUNNING,
        STOPPED,
        REVERSING
    }
    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
    public INTAKE_MOTOR_STATE intakeMotorPrevState = INTAKE_MOTOR_STATE.STOPPED;

    // TODO: Update these values
    public enum STACK_INTAKE_LIFT_STATE {
        LIFTED(0.62), //0.63
        DROPPED(0.20),
        CLIMBED(0.57);//0.50

        private double liftPosition;

        STACK_INTAKE_LIFT_STATE(double liftPosition){
            this.liftPosition = liftPosition;
        }
        public double getLiftPosition(){
            return liftPosition;
        }

    }

    public STACK_INTAKE_LIFT_STATE stackIntakeLiftState = STACK_INTAKE_LIFT_STATE.DROPPED;

    public double intakeMotorPower = 0.64;//0.70
    public double reverseMotorPower = 0.75;

    public Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        stackIntakeLiftServo = hardwareMap.get(Servo.class, "intake_lift_servo");
        stackIntakeServoLeft = hardwareMap.get(CRServo.class, "horiz_servo_left");
        stakeIntakeServoRight = hardwareMap.get(CRServo.class, "horiz_servo_right");
        initIntake();
    }

    public void initIntake(){
        intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
        intakeMotor.setPower(0);
        moveRollerHeight(STACK_INTAKE_LIFT_STATE.LIFTED);
    }

    public void moveRollerHeight(STACK_INTAKE_LIFT_STATE targetIntakeRollerHeight){
        stackIntakeLiftServo.setPosition(targetIntakeRollerHeight.liftPosition);
        stackIntakeLiftState = targetIntakeRollerHeight;
    }

    public void moveIntakeLiftUp(){
        moveRollerHeight(STACK_INTAKE_LIFT_STATE.LIFTED);
    }
    public void moveIntakeLiftDown(){
        moveRollerHeight(STACK_INTAKE_LIFT_STATE.DROPPED);
    }

    public void moveIntakeLiftClimbed(){
        moveRollerHeight(STACK_INTAKE_LIFT_STATE.CLIMBED);
    }

    public void startIntakeInward(){
        if(intakeMotorState != INTAKE_MOTOR_STATE.RUNNING){
            runIntakeMotor(DcMotor.Direction.FORWARD, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.RUNNING;
            intakeMotorPrevState = intakeMotorState;

        }
    }

    public void reverseIntake() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            runIntakeMotor(DcMotor.Direction.REVERSE, reverseMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
            intakeMotorPrevState = intakeMotorState;
        }
    }

    public void stopIntake() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.STOPPED) {
             runIntakeMotor(DcMotorSimple.Direction.FORWARD, 0.0);
             intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
             intakeMotorPrevState = intakeMotorState;
        }
    }

    public void runIntakeMotor(DcMotor.Direction direction, double intakePower) {
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(intakePower);
    }

    public void startStackIntakeToCollect(){
        if(stackIntakeLiftState == STACK_INTAKE_LIFT_STATE.DROPPED) {
            stackIntakeTimer.reset();
            stackIntakeServoLeft.setPower(1);
            stakeIntakeServoRight.setPower(-0.8);//0.8
            stackIntakeActivated = true;
            stackIntakeServoState = STACK_INTAKE_SERVO_STATE.COLLECT;
        }
    }

    public void runStackIntakeOneRotation(){
        if(stackIntakeActivated){
            if (stackIntakeTimer.time()>700){ //1300
                stopStackIntake();
            }
        }
    }
    public void runStackIntakeOneRotationAuto(){
        startStackIntakeToCollect();
        if(stackIntakeActivated){
            while (stackIntakeTimer.time()<550){ //550
            }
            stopStackIntake();
        }
    }

    public void reverseStackIntake(){
        if(stackIntakeLiftState == STACK_INTAKE_LIFT_STATE.DROPPED) {
            reverseStackIntakeTimer.reset();
            stackIntakeServoLeft.setPower(-0.8);//0.8
            stakeIntakeServoRight.setPower(1);
            reverseStackIntakeFlag = true;
            stackIntakeServoState = STACK_INTAKE_SERVO_STATE.REVERSE;
        }
    }

    public void reverseStackIntakeTele(){
        if(stackIntakeLiftState == STACK_INTAKE_LIFT_STATE.DROPPED) {
            //reverseStackIntakeTimer.reset();
            stackIntakeServoLeft.setPower(-0.8);
            stakeIntakeServoRight.setPower(1);
           // reverseStackIntakeFlag = true;
            stackIntakeServoState = STACK_INTAKE_SERVO_STATE.REVERSE;
        }
    }

    public void runReverseStackIntakeOneRotation(){
        if(reverseStackIntakeFlag){
            if (reverseStackIntakeTimer.time() > 700){ //850
                stopStackIntake();
            }
        }
    }

    public void runReverseStackIntakeOneRotationAuto(){
        reverseStackIntake();
        if(reverseStackIntakeFlag){
            while(reverseStackIntakeTimer.time() < 350){ //850
            }
            stopStackIntake();
        }
    }

    public void stopStackIntake(){
        stackIntakeServoLeft.setPower(0);
        stakeIntakeServoRight.setPower(0);
        stackIntakeActivated = false;
        stackIntakeServoState = STACK_INTAKE_SERVO_STATE.STOPPED;
    }

    public void stopStackIntakeTele(){
        stackIntakeServoLeft.setPower(0);
        stakeIntakeServoRight.setPower(0);
        //stackIntakeActivated = false;
        stackIntakeServoState = STACK_INTAKE_SERVO_STATE.STOPPED;
    }

    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Intake");
        telemetry.addData("    State", getIntakeState());
        telemetry.addData("    Motor Power", intakeMotorPower);
        telemetry.addData("    Stack Intake Lift State", stackIntakeLiftState);
        telemetry.addData("    Stack Intake Lift Position", stackIntakeLiftServo.getPosition());
        telemetry.addData("    Stack Intake Servo Left:", stackIntakeServoLeft);
        telemetry.addData("    Stack Intake Servo Right:", stakeIntakeServoRight);
        telemetry.addData("Stack Servo State: ", stackIntakeServoState);
        telemetry.addLine("=============");
    }

}