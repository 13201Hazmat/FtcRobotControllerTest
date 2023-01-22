package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.GameOpModes.GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class OuttakeArm {
    //Initialization of <outtake arm servo's>
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;
    public Servo outtakeArmLeft;
    public Servo outtakeArmRight;

    public NormalizedColorSensor outtakeWristColor;
    public NormalizedColorSensor outtakeGripColor;

    public enum OUTTAKE_ARM_STATE{
        TRANSFER(0.98, 0.02), //TODO test real values
        DROP(0.4, 0.6), //TODO test real values
        LOW_JUNCTION (0.2, 0.8),
        AUTO_HIGH_JUNCTION(0.35,0.65),
        AUTO_MEDIUM_JUNCTION(0.2,0.8);

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
        WRIST_TRANSFER(0.40), //TODO test real, 0.36
        WRIST_DROP(0.48), //0.52 TODO test real
        WRIST_AUTO_DROP(0.56),//0.52
        WRIST_LOW_JUNCTION(0.68), //TODO test real
        WRIST_MIN(0.2),
        WRIST_MAX(0.76);

        private double wristPosition;

        OUTTAKE_WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }
    public OUTTAKE_WRIST_STATE outtakeWristState = OUTTAKE_WRIST_STATE.WRIST_TRANSFER;
    public double OUTTAKE_WRIST_DELTA = 0.02;

    //Initialization of GRIP_STATE
    public enum OUTTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.40),
        CLOSED(0.75); //Max 1.0

        private double gripPosition;

        OUTTAKE_GRIP_STATE(double gripPosition){
            this.gripPosition = gripPosition;
        }
        public double getGripPosition(){
            return gripPosition;
        }
    }
    public OUTTAKE_GRIP_STATE outtakeGripState = OUTTAKE_GRIP_STATE.CLOSED;
    //constants for Hand and grip position
    public enum OUTTAKE_GRIP_COLOR_SENSOR_STATE {
        DETECTED,
        NOT_DETECTED
    }
    public OUTTAKE_GRIP_COLOR_SENSOR_STATE outtakeGripColorSensorState = OUTTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;
    public boolean autoIntakeClose = true;

    public OuttakeArm(HardwareMap hardwareMap) { //map hand servo's to each

        outtakeWristServo = hardwareMap.get(Servo.class, "outtake_wrist_servo");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtake_grip_servo");

        outtakeWristColor = hardwareMap.get(NormalizedColorSensor.class, "outtake_wrist_sensor");
        outtakeGripColor = hardwareMap.get(NormalizedColorSensor.class, "outtake_grip_sensor");

        outtakeArmLeft = hardwareMap.get(Servo.class, "outtake_arm_left");
        outtakeArmRight = hardwareMap.get(Servo.class, "outtake_arm_right");

        initOuttakeArm();
    }

    //initialize outtakeArm
    public void initOuttakeArm() {
        moveArm(OUTTAKE_ARM_STATE.TRANSFER);

        if (GameField.opModeRunning == HAZMAT_AUTONOMOUS) {
            closeGrip();
        }
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

    public double outtakeGripDistance;
    /**
     * Returns the color sensor state back, and sets specific values to check if the sensor
     * is detecting anything
     * @return
     */
    public boolean senseOuttakeCone(){
        boolean outtakeConeSensed = false;
        if (outtakeWristState == OUTTAKE_WRIST_STATE.WRIST_TRANSFER) {
            if (outtakeGripColor instanceof DistanceSensor) {
                outtakeGripDistance = ((DistanceSensor) outtakeGripColor).getDistance(DistanceUnit.MM);
            }
            if (outtakeGripDistance < 60.0) {
                outtakeConeSensed = true;
            } else {
                outtakeConeSensed = false;
            }
        }
        return outtakeConeSensed;
    }

    public void moveWrist(OUTTAKE_WRIST_STATE toWristState){
        outtakeWristServo.setPosition(toWristState.wristPosition);
        outtakeWristState = toWristState;
    }
    /////TODO : RUN WRIST ERROR

    public boolean runWristInDropFlag = false;
    //public ElapsedTime runWristBasedOnArmTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    public void moveArm(OUTTAKE_ARM_STATE toArmState){
        outtakeArmLeft.setPosition(toArmState.leftArmPosition);
        outtakeArmRight.setPosition(toArmState.rightArmPosition);
        outtakeArmState = toArmState;
        if(outtakeArmState == OUTTAKE_ARM_STATE.TRANSFER){
            moveWrist(OUTTAKE_WRIST_STATE.WRIST_TRANSFER);
            openGrip();
        }
        runWristInDropFlag = true;
    }

    /*public void runWristBasedOnArm() {
        if (runWristBasedOnArmFlag) {
            if (runWristBasedOnArmTimer.time() > 300) {
                if (outtakeArmState == OUTTAKE_ARM_STATE.DROP) {
                    moveWrist(OUTTAKE_WRIST_STATE.WRIST_DROP);
                } else if (outtakeArmState == OUTTAKE_ARM_STATE.LOW_JUNCTION) {
                    moveWrist(OUTTAKE_WRIST_STATE.WRIST_LOW_JUNCTION);
                }
                runWristBasedOnArmFlag = false;
            }
        }
    }*/



    public double outtakeWristDistance;
    public boolean senseJunction(){
        boolean junctionSensed = false;
        if (outtakeWristState == OUTTAKE_WRIST_STATE.WRIST_DROP) {
            if (outtakeWristColor instanceof DistanceSensor) {
                outtakeWristDistance = ((DistanceSensor) outtakeWristColor).getDistance(DistanceUnit.MM);
            }

            if (outtakeWristDistance < 60) {
                junctionSensed = true;
            } else {
                junctionSensed = false;
            }
        }
        return junctionSensed;
    }

    public void moveOuttakeWristUp() {
        if (outtakeWristServo.getPosition() <= outtakeWristState.WRIST_MAX.getWristPosition()) {
            outtakeWristServo.setPosition(outtakeWristServo.getPosition() + OUTTAKE_WRIST_DELTA);
        } else {
            outtakeWristServo.setPosition(outtakeWristState.WRIST_MAX.getWristPosition());
        }
    }

    public void moveOuttakeWristDown() {
        if (outtakeWristServo.getPosition() >= outtakeWristState.WRIST_MIN.getWristPosition()) {
            outtakeWristServo.setPosition(outtakeWristServo.getPosition() - OUTTAKE_WRIST_DELTA);
        } else {
            outtakeWristServo.setPosition(outtakeWristState.WRIST_MIN.getWristPosition());
        }
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

    public double isOuttakeGripInStateError = 0;
    public boolean isOuttakeGripInState(OUTTAKE_GRIP_STATE toOuttakeGripState) {
        isOuttakeGripInStateError = Math.abs(outtakeGripServo.getPosition() - toOuttakeGripState.getGripPosition());
        return (outtakeGripState == toOuttakeGripState && isOuttakeGripInStateError <=0.03);
    }

}
