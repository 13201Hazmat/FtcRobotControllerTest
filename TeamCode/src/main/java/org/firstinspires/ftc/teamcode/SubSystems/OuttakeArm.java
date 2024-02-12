package org.firstinspires.ftc.teamcode.SubSystems;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.GameOpModes.OldAuto.GameField;

public class OuttakeArm {
    //Initialization of <outtake arm servo's>
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;
    public Servo outtakeArmLeft;
    public Servo outtakeArmRight;

    public Servo outtakeAlignment;
    public enum OUTTAKE_ARM_STATE{
        ZERO(0.94,0.06),
        //TRAVEL(0.76,0.24),
        TRANSFER(0.97,0.03), //0.95,0.05
        PICKUP(0.97,0.03), //0.95,0.05
        READY_FOR_TRANSFER(0.78,0.22),//0.80, 0.20
        //DROP(0.21,0.79); //1,0
        DROP_LOWEST(0.21,0.79),
        DROP_LOW_LINE(0.21,0.79),
        DROP_BELOW_MID(0.23,0.77),
        DROP_LEVEL_MID(0.26,0.74),
        DROP_BELOW_HIGH(0.28,0.72),
        DROP_LEVEL_HIGH(0.33,0.67),
        DROP_HIGHEST(0.37,0.63);

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
        TRANSFER(0.03),//0.025
        PICKUP(0.05), //0.06
        READY_FOR_TRANSFER(0.0),//0.0
        DROP(0.90); //0.94

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

    public enum OUTTAKE_ALIGN_STATE{
        UP(0.0),
        DOWN(0.25);

        private double alignPosition;

        OUTTAKE_ALIGN_STATE(double alignPosition){
            this.alignPosition = alignPosition;
        }

        public double getAlignPosition() {
            return alignPosition;
        }
    }
    public OUTTAKE_ALIGN_STATE outtakeAlignState = OUTTAKE_ALIGN_STATE.UP;

    public Telemetry telemetry;
    public OuttakeArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist_servo");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip_servo");

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtake_arm_left");
        outtakeArmRight = hardwareMap.get(Servo.class, "outtake_arm_right");

        outtakeAlignment = hardwareMap.get(Servo.class, "outtake_align");

        initOuttakeArm();
    }



    //initialize outtakeArm
    public void initOuttakeArm() {
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_CALIBRATE_OUTTAKE
                /*|| GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS*/) {
            moveArm(OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
            moveWrist(OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        } else {
            moveArm(OUTTAKE_ARM_STATE.READY_FOR_TRANSFER);
            moveWrist(OUTTAKE_WRIST_STATE.READY_FOR_TRANSFER);
        }
        closeGrip();
        backPlateAlignUp();

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

    public void backPlateAlignUp(){
        outtakeAlignment.setPosition(OUTTAKE_ALIGN_STATE.UP.alignPosition);
        outtakeAlignState = OUTTAKE_ALIGN_STATE.UP;
    }

    public void backPlateAlignDown(){
        outtakeAlignment.setPosition(OUTTAKE_ALIGN_STATE.DOWN.alignPosition);
        outtakeAlignState = OUTTAKE_ALIGN_STATE.DOWN;
    }

    ElapsedTime pixelDropTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void dropOnePixel(){
        openGrip();
        pixelDropTimer.reset();
        while (pixelDropTimer.time() <150) { //100
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
        while (pixelDropTimer.time() <150) { //100
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