package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
    public Servo intakeLiftServo;
    public CRServo horizServoLeft, horizServoRight;

    public boolean stackIntakeActivated = false;
    boolean reverseIntakeHorizFlag = false;

    public ElapsedTime horizIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public ElapsedTime reverseHorizIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public enum HORIZ_SERVO_STATE{
        COLLECT,
        REVERSE,
        STOPPED
    }
    public HORIZ_SERVO_STATE horizServoState = HORIZ_SERVO_STATE.STOPPED;

    public enum INTAKE_MOTOR_STATE{
        RUNNING,
        STOPPED,
        REVERSING
    }
    public INTAKE_MOTOR_STATE intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
    public INTAKE_MOTOR_STATE intakeMotorPrevState = INTAKE_MOTOR_STATE.STOPPED;

    // TODO: Update these values
    public enum INTAKE_ROLLER_HEIGHT{
        LIFTED(0.62), //0.63
        DROPPED(0.20),
        CLIMBED(0.57);//0.50

        private double liftPosition;

        INTAKE_ROLLER_HEIGHT(double liftPosition){
            this.liftPosition = liftPosition;
        }
        public double getLiftPosition(){
            return liftPosition;
        }

    }

    public INTAKE_ROLLER_HEIGHT intakeRollerHeightState = INTAKE_ROLLER_HEIGHT.DROPPED;

    public double intakeMotorPower = 0.75;//1

    public Telemetry telemetry;
    public Intake(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        intakeLiftServo = hardwareMap.get(Servo.class, "intake_lift_servo");
        horizServoLeft = hardwareMap.get(CRServo.class, "horiz_servo_left");
        horizServoRight = hardwareMap.get(CRServo.class, "horiz_servo_right");
        initIntake();
    }

    public void initIntake(){
        intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
        intakeMotor.setPower(0);
        moveRollerHeight(INTAKE_ROLLER_HEIGHT.LIFTED);
    }

    public void moveRollerHeight(INTAKE_ROLLER_HEIGHT targetIntakeRollerHeight){
        intakeLiftServo.setPosition(targetIntakeRollerHeight.liftPosition);
        intakeRollerHeightState = targetIntakeRollerHeight;
    }

    public void moveIntakeLiftUp(){
        moveRollerHeight(INTAKE_ROLLER_HEIGHT.LIFTED);
    }
    public void moveIntakeLiftDown(){
        moveRollerHeight(INTAKE_ROLLER_HEIGHT.DROPPED);
    }

    public void moveIntakeLiftClimber(){
        moveRollerHeight(INTAKE_ROLLER_HEIGHT.CLIMBED);
    }

    public void toggleStackIntake(){
        if(horizServoState != HORIZ_SERVO_STATE.COLLECT){
            startIntakeInward();
        } else {
            stopIntake();
        }
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
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower);
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

    public void stopHorizIntakeInward(){
        horizServoLeft.setPower(0);
        horizServoRight.setPower(0);
        stackIntakeActivated = false;
        horizServoState = HORIZ_SERVO_STATE.STOPPED;
    }

    public void stopHorizIntakeReverse(){
        horizServoLeft.setPower(0);
        horizServoRight.setPower(0);
        reverseIntakeHorizFlag = false;
        horizServoState = HORIZ_SERVO_STATE.STOPPED;
    }

    public void runIntakeMotor(DcMotor.Direction direction, double intakePower) {
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(intakePower);
    }

    public void startIntakeHorizToCollect(){
        if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
            horizIntakeTimer.reset();
            horizServoLeft.setPower(1);
            horizServoRight.setPower(-0.8);
            stackIntakeActivated = true;
            horizServoState = HORIZ_SERVO_STATE.COLLECT;
        }
    }
    public void runHorizIntakeRotation(){
        if(stackIntakeActivated){
            if(horizIntakeTimer.time() > 850){ //1300
                stopHorizIntakeInward();
            }
        }
    }

    public void reverseIntakeHoriz(){
        if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
            reverseHorizIntakeTimer.reset();
            horizServoLeft.setPower(-0.8);
            horizServoRight.setPower(1);
            reverseIntakeHorizFlag = true;
            horizServoState = HORIZ_SERVO_STATE.REVERSE;
        }
    }

    public void runReverseIntakeHorizRotation(){
        if(reverseIntakeHorizFlag){
            if(reverseHorizIntakeTimer.time() > 500){ //850
                stopHorizIntakeReverse();
            }
        }
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
        telemetry.addData("    Horizontal Servo 2:", horizServoLeft);
        telemetry.addData("    Horizontal Servo 2:", horizServoRight);
        telemetry.addData("    Motor Power", intakeMotorPower);
        telemetry.addLine("=============");
    }

}