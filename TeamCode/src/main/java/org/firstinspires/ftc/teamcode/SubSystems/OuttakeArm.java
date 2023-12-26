package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class OuttakeArm {
    //Initialization of <outtake arm servo's>
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;
    public Servo outtakeArmLeft;
    public Servo outtakeArmRight;

    public enum OUTTAKE_ARM_STATE{
        ZERO(0.94,0.06),
        //TRAVEL(0.76,0.24),
        TRANSFER(0.95,0.05),
        PICKUP(0.95,0.05),
        READY_FOR_TRANSFER(0.76,0.24),
        DROP(0.21,0.78); //1,0

        private double leftArmPosition;
        private double rightArmPosition;

        OUTTAKE_ARM_STATE(double leftArmPosition, double rightArmPosition){
            this.leftArmPosition = leftArmPosition;
            this.rightArmPosition = rightArmPosition;
        }
        public double getLeftArmPosition(){
            return leftArmPosition;
        }
        public double getRightArmPosition(){
            return rightArmPosition;
        }
    }

    public OUTTAKE_ARM_STATE outtakeArmState = OUTTAKE_ARM_STATE.TRANSFER;


    //Hand - wrist, grip state declaration
    public enum OUTTAKE_WRIST_STATE {
        ZERO(0),
        //TRAVEL(0.03), //0.48
        TRANSFER(0.02),//0.12
        PICKUP(0.04), //0.16
        READY_FOR_TRANSFER(0.02),//0.09
        DROP(0.88); //0.94

        private double wristPosition;

        OUTTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }
    public OUTTAKE_WRIST_STATE outtakeWristState = OUTTAKE_WRIST_STATE.TRANSFER;

    //Initialization of GRIP_STATE
    public enum OUTTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.8),
        CLOSED(0.47);

        private double gripPosition;

        OUTTAKE_GRIP_STATE(double gripPosition){
            this.gripPosition = gripPosition;
        }
        public double getGripPosition(){
            return gripPosition;
        }
    }
    public OUTTAKE_GRIP_STATE outtakeGripState = OUTTAKE_GRIP_STATE.CLOSED;

    public Telemetry telemetry;
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist_servo");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip_servo");

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtake_arm_left");
        outtakeArmRight = hardwareMap.get(Servo.class, "outtake_arm_right");

        initOuttakeArm();
    }



    //initialize outtakeArm
    public void initOuttakeArm() {
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_CALIBRATE_OUTTAKE ||
                GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            moveArm(OUTTAKE_ARM_STATE.TRANSFER);
            moveWrist(OUTTAKE_WRIST_STATE.TRANSFER);
        } else {
            moveArm(OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
            moveWrist(OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        }
        closeGrip();
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip() {
        outtakeGripServo.setPosition(OUTTAKE_GRIP_STATE.OPEN.gripPosition);
        outtakeGripState = OUTTAKE_GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        outtakeGripServo.setPosition(OUTTAKE_GRIP_STATE.CLOSED.gripPosition);
        outtakeGripState = OUTTAKE_GRIP_STATE.CLOSED;
    }

    public void toggleGrip(){
        if (outtakeGripState == OUTTAKE_GRIP_STATE.OPEN) {
            closeGrip();
        } else {
            openGrip();
        }
    }

    ElapsedTime pixelDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void dropOnePixel(){
        openGrip();
        pixelDropTimer.reset();
        while (pixelDropTimer.time() <160) { //100
            //gamepadcontroller.runbyGamepadcontroller
        };
        closeGrip();
        while (pixelDropTimer.time() <250) { //200
            //gamepadcontroller.runbyGamepadcontroller
        };
    }

    public void autoDropOnePixel(){
        openGrip();
        pixelDropTimer.reset();
        while (pixelDropTimer.time() <160) { //100
            //gamepadcontroller.runbyGamepadcontroller
        };
        closeGrip();
        while (pixelDropTimer.time() <250) { //200
            //gamepadcontroller.runbyGamepadcontroller
        };
    }

    public void moveWrist(OUTTAKE_WRIST_STATE toWristState){
        outtakeWristServo.setPosition(toWristState.wristPosition);
        outtakeWristState = toWristState;
    }

    public void rotateWrist(double direction) {
        double newWristPosition = outtakeWristServo.getPosition() + direction*0.01;
        if (newWristPosition >=0.0 && newWristPosition <=1.0) {
            outtakeWristServo.setPosition(newWristPosition);
        }
    }

    public void zeroWrist(){
        moveWrist(OUTTAKE_WRIST_STATE.ZERO);
    }

    public ElapsedTime wristTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void moveArm(OUTTAKE_ARM_STATE toArmState) { //UPDATE FOR THIS YEAR!!
        outtakeArmLeft.setPosition(toArmState.leftArmPosition);
        outtakeArmRight.setPosition(toArmState.rightArmPosition);
        outtakeArmState = toArmState;
    }

    public void rotateArm(double direction) {
        double newPositionLeft = outtakeArmLeft.getPosition() + direction *0.01;
        double newPositionRight = outtakeArmRight.getPosition() - direction *0.01;
        if (newPositionLeft >= 0.0 && newPositionLeft <= 1.0 &&
                newPositionRight >= 0.0 && newPositionRight <= 1.0) {
            outtakeArmLeft.setPosition(newPositionLeft);
            outtakeArmRight.setPosition(newPositionRight);
        }
    }

    public void zeroArm(){
        moveArm(OUTTAKE_ARM_STATE.ZERO);
    }

    public double isOuttakeArmInStateError = 0;
    public boolean isOuttakeArmInState(OUTTAKE_ARM_STATE toOuttakeArmState) {
        isOuttakeArmInStateError = Math.abs(outtakeArmLeft.getPosition() - toOuttakeArmState.leftArmPosition);
        return (outtakeArmState == toOuttakeArmState && isOuttakeArmInStateError <= 0.02);
    }

    public double isOuttakeWristInStateError = 0;
    public boolean isOuttakeWristInState(OUTTAKE_WRIST_STATE toOuttakeWristState) {
        isOuttakeWristInStateError = Math.abs(outtakeWristServo.getPosition() - toOuttakeWristState.getWristPosition());
        return (outtakeWristState == toOuttakeWristState && isOuttakeWristInStateError <= 0.02);
    }


    public void printDebugMessages() {
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Outtake Arm");
        telemetry.addData("   State", outtakeArmState);
        telemetry.addData("   Left Servo position", outtakeArmLeft.getPosition());
        telemetry.addData("   Right Servo position", outtakeArmRight.getPosition());
        telemetry.addLine("Outtake Wrist");
        telemetry.addData("   State", outtakeWristState);
        telemetry.addData("   Wrist Servo position", outtakeWristServo.getPosition());
        telemetry.addLine("Outtake Grip");
        telemetry.addData("   State", outtakeWristState);
        telemetry.addData("   Grip Servo position", outtakeGripServo.getPosition());


    }
}