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

    public ElapsedTime horizIntakeTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
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
        LIFTED(0.65), //0.63
        DROPPED(0.14);//0.15, 1

        private double liftPosition;

        INTAKE_ROLLER_HEIGHT(double liftPosition){
            this.liftPosition = liftPosition;
        }
        public double getLiftPosition(){
            return liftPosition;
        }

        /*
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
         */

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
        //if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.LIFTED);
        //}
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

    public void toggleStackIntake(){
        if(horizServoState != HORIZ_SERVO_STATE.COLLECT){
            startIntakeInward();
        } else {
            stopIntake();
        }
    }

    /*
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
     */

    public void startIntakeInward(){
        if(intakeMotorState != INTAKE_MOTOR_STATE.RUNNING){
            /*
            if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED){
                moveIntakeHorizToCollect();
            }
             */
            runIntakeMotor(DcMotor.Direction.FORWARD, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.RUNNING;
            intakeMotorPrevState = intakeMotorState;

        }
    }

    public void reverseIntake() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            /*
            if (intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
                moveIntakeHorizToReverse();
            }
             */
            //moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_DROPPED);
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
            intakeMotorPrevState = intakeMotorState;
        }
    }

    public void reverseIntakeTeleOp() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            /*
            if (intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
                moveIntakeHorizToReverse();
            }
             */
            //moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_DROPPED);
            runIntakeMotor(DcMotor.Direction.REVERSE, intakeMotorPower);
            intakeMotorState = INTAKE_MOTOR_STATE.REVERSING;
            intakeMotorPrevState = intakeMotorState;
        }
    }

    public void reverseIntakeForPurplePixelDrop() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.REVERSING) {
            moveRollerHeight(INTAKE_ROLLER_HEIGHT.DROPPED);
            reverseIntakeHoriz();
        }
    }

    public void stopIntake() {
        if(intakeMotorState != INTAKE_MOTOR_STATE.STOPPED) {
            /*if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED){
                horizServoLeft.setPower(0.0);
                horizServoRight.setPower(0.0);
             */
                runIntakeMotor(DcMotorSimple.Direction.FORWARD, 0.0);
                //horizServoState = HORIZ_SERVO_STATE.STOPPED;
                intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
             intakeMotorPrevState = intakeMotorState;
            /*} else {runIntakeMotor(DcMotorSimple.Direction.FORWARD, 0.0);
                intakeMotorPrevState = intakeMotorState;
                intakeMotorState = INTAKE_MOTOR_STATE.STOPPED;
            }
             */
        }
    }

    public void stopHorizIntake(){
        horizServoLeft.setPower(0);
        horizServoRight.setPower(0);
        stackIntakeActivated = false;
        horizServoState = HORIZ_SERVO_STATE.STOPPED;
    }

    public void runIntakeMotor(DcMotor.Direction direction, double intakePower) {
        intakeMotor.setDirection(direction);
        intakeMotor.setPower(intakePower);
    }

    public void runHorizIntakeRotation(){
        if(stackIntakeActivated){
            if(horizIntakeTimer.time() > 1300){ //850
                stopHorizIntake();
            }
        }
    }
    public void startIntakeHorizToCollect(){
        if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
            horizIntakeTimer.reset();
            horizServoLeft.setPower(1);
            horizServoRight.setPower(-1);
            stackIntakeActivated = true;
            horizServoState = HORIZ_SERVO_STATE.COLLECT;
        }
    }
    public void reverseIntakeHoriz(){
        if(intakeRollerHeightState == INTAKE_ROLLER_HEIGHT.DROPPED) {
            horizIntakeTimer.reset();
            horizServoLeft.setPower(-1);
            horizServoRight.setPower(1);
            stackIntakeActivated = true;
            horizServoState = HORIZ_SERVO_STATE.REVERSE;
        }
    }
    public INTAKE_MOTOR_STATE getIntakeState() {
        return intakeMotorState;
    }

    public Action moveIntakeHorizToCollectAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                startIntakeHorizToCollect();
                return false;
            }
        };
    }

    public Action moveIntakeHorizToReverseAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                reverseIntakeHoriz();
                return false;
            }
        };
    }

    public Action stopIntakeMotorAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                stopIntake();
                return false;
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
                return false;
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
                return false;
            }
        };
    }

    public Action moveRollerHeightLiftedAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.LIFTED);
                return false;
            }
        };
    }

    public Action moveRollerHeightDroppedAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.DROPPED);
                return false;
            }
        };
    }

    /*
    public Action moveRollerHeightAboveStackAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                moveRollerHeight(INTAKE_ROLLER_HEIGHT.INTAKE_ROLLER_HEIGHT_ABOVE_STACK);
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
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
                return false;
            }
        };
    }
     */



    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Intake");
        telemetry.addData("    State", getIntakeState());
        telemetry.addData("    Roller Height State", intakeRollerHeightState);
        telemetry.addData("    Roller Servo Position", intakeLiftServo.getPosition());
        telemetry.addData("Horizontal Servo 2:", horizServoLeft);
        telemetry.addData("Horizontal Servo 2:", horizServoRight);
        telemetry.addData("    Motor Power", intakeMotorPower);
        telemetry.addLine("=============");
    }

}