package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
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
        INTAKE_ROLLER_INIT_AUTO(0.68,7),
        INTAKE_ROLLER_HEIGHT_ABOVE_STACK(0.39,6),
        INTAKE_ROLLER_LIFTED_5(0.36,5), //0.355
        INTAKE_ROLLER_LIFTED_4(0.325,4), //0.32
        INTAKE_ROLLER_LIFTED_3(0.285,3), //0.28
        INTAKE_ROLLER_LIFTED_2(0.245,2), //0.24
        INTAKE_ROLLER_DROPPED(0.19,1);

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
            if (ord >7) ord = 7;
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
        //if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_INIT_AUTO);
        //}
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

    public void moveIntakeRollerOnePixelUp(){
        if (intakeRollerHeightState != INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED) {
            moveRollerHeight(intakeRollerHeightState.byIndex((intakeRollerHeightState.getIndex()+1)));
        }
    }

    public void moveIntakeRollerToLevel(int level){
        if (level > 6) {
            level = 6;
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
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING;
        }
    }

    public void reverseIntakeForPurplePixelDrop() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.INTAKE_MOTOR_REVERSING) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
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

    public Action stopIntakeMotorAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                stopIntakeMotor();
                return true;
            }
        };
    }

    public Action reverseIntakeAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                reverseIntake();
                return true;
            }
        };
    }


    public Action startIntakeInwardAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                startIntakeInward();
                return true;
            }
        };
    }

    public Action moveRollerHeightAboveStackAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
                return true;
            }
        };
    }

    public Action moveRollerHeightLevel5Action(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_5);
                return true;
            }
        };
    }

    public Action moveRollerHeightLevel4Action(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_4);
                return true;
            }
        };
    }

    public Action moveRollerHeightLevel3Action(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_3);
                return true;
            }
        };
    }

    public Action moveRollerHeightLevel2Action(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_LIFTED_2);
                return true;
            }
        };
    }

    public Action moveRollerHeightDroppedAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_DROPPED);
                return true;
            }
        };
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