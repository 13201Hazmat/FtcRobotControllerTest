package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.teamcode.GameOpModes.GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

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
        TRANSFER(1.0, 0.0), //TODO test real values
        DROP(0.4, 0.6); //TODO test real values

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
    public enum WRIST_STATE {
        WRIST_TRANSFER(0.33), //TODO test real, 0.36
        WRIST_DROP(0.56), //0.56 TODO test real
        WRIST_LOW_JUNCTION(0.34); //TODO test real

        private double wristPosition;

        WRIST_STATE(double wristPosition){
            this.wristPosition = wristPosition;
        }
        public double getWristPosition(){
            return wristPosition;
        }
    }
    public WRIST_STATE wristState = WRIST_STATE.WRIST_TRANSFER;
    public double OUTTAKE_WRIST_DELTA = 0.02;

    //Initialization of GRIP_STATE
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN(0.5),
        CLOSED(0.75); //Max 1.0

        private double gripPosition;

        GRIP_STATE(double gripPosition){
            this.gripPosition = gripPosition;
        }
        public double getGripPosition(){
            return gripPosition;
        }
    }
    public GRIP_STATE gripState = GRIP_STATE.CLOSED;
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
        if (outtakeWristColor instanceof SwitchableLight) {
            ((SwitchableLight)outtakeWristColor).enableLight(true);
        }
        if (outtakeGripColor instanceof SwitchableLight) {
            ((SwitchableLight)outtakeGripColor).enableLight(true);
        }

        moveArm(OUTTAKE_ARM_STATE.TRANSFER);

        if (GameField.opModeRunning == HAZMAT_AUTONOMOUS) {
            closeGrip();
        }
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip() {
        outtakeGripServo.setPosition(GRIP_STATE.OPEN.gripPosition);
        gripState = GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        outtakeGripServo.setPosition(GRIP_STATE.CLOSED.gripPosition);
        gripState = GRIP_STATE.CLOSED;
    }

    public double outtakeGripDistance;
    /**
     * Returns the color sensor state back, and sets specific values to check if the sensor
     * is detecting anything
     * @return
     */
    public boolean senseOuttakeCone(){
        boolean outtakeConeSensed = false;
        if (wristState == WRIST_STATE.WRIST_TRANSFER) {
            if (outtakeGripColor instanceof DistanceSensor) {
                outtakeGripDistance = ((DistanceSensor) outtakeGripColor).getDistance(DistanceUnit.MM);
            }

            if (outtakeGripDistance < 25) {
                outtakeConeSensed = true;
            } else {
                outtakeConeSensed = false;
            }
        }
        return outtakeConeSensed;
    }

    public void moveWrist(WRIST_STATE toWristState){
        outtakeWristServo.setPosition(toWristState.wristPosition);
        wristState = toWristState;
    }

    public void moveArm(OUTTAKE_ARM_STATE toArmState){
        outtakeArmLeft.setPosition(toArmState.leftArmPosition);
        outtakeArmRight.setPosition(toArmState.rightArmPosition);
        outtakeArmState = toArmState;
        if(outtakeArmState == OUTTAKE_ARM_STATE.TRANSFER){
            moveWrist(WRIST_STATE.WRIST_TRANSFER);
            openGrip();
        } else {
            moveWrist(WRIST_STATE.WRIST_DROP);
        }

    }

    public boolean isOuttakeArmInState(OUTTAKE_ARM_STATE outtakeArmState) {
        return (outtakeArmLeft.getPosition() == outtakeArmState.leftArmPosition);
    }

    public double outtakeWristDistance;
    public boolean senseJunction(){
        boolean junctionSensed = false;
        if (wristState == WRIST_STATE.WRIST_DROP) {
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
        //TODO : Put Limits
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() + OUTTAKE_WRIST_DELTA);
    }

    public void moveOuttakeWristDown() {
        //TODO : Put Limits
        outtakeWristServo.setPosition(outtakeWristServo.getPosition() - OUTTAKE_WRIST_DELTA);
    }

}
