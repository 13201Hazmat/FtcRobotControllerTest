package org.firstinspires.ftc.teamcode.SubSystems;

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
        ZERO(1,0),
        TRAVEL(0.85,0.15),
        TRANSFER(1,0),
        PICKUP(1,0),
        READY_FOR_TRANSFER(0.9,0.1),
        DROP(0.07,0.93);

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
        TRAVEL(0.5),
        TRANSFER(0),
        PICKUP(0.02),
        READY_FOR_TRANSFER(0),
        DROP(0.85);

        private double wristPosition;

        OUTTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }
    public OUTTAKE_WRIST_STATE outtakeWristState = OUTTAKE_WRIST_STATE.TRANSFER;
    public double OUTTAKE_WRIST_DELTA = 0.02; //UP

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
        moveArm(OUTTAKE_ARM_STATE.TRAVEL);
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
        while (pixelDropTimer.time() <200) {
            //gamepadcontroller.runbyGamepadcontroller
        };
        closeGrip();
        while (pixelDropTimer.time() <200) {
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

    public void moveArm(OUTTAKE_ARM_STATE toArmState) { //UPDATE FOR THIS YEAR!!
        outtakeArmLeft.setPosition(toArmState.leftArmPosition);
        outtakeArmRight.setPosition(toArmState.rightArmPosition);
        outtakeArmState = toArmState;

        switch (outtakeArmState) {
            case TRAVEL:
                openGrip();
                moveWrist(OUTTAKE_WRIST_STATE.TRAVEL);
                break;
            case TRANSFER:
                openGrip();
                moveWrist(OUTTAKE_WRIST_STATE.TRANSFER);
                break;
            case PICKUP:
                moveWrist(OUTTAKE_WRIST_STATE.PICKUP);
                closeGrip();
                break;
            case READY_FOR_TRANSFER:
                moveWrist(OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
                break;
            case DROP:
                moveWrist(OUTTAKE_WRIST_STATE.DROP);
                break;
        }
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