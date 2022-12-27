package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public Servo intakeGripServo;

    public NormalizedColorSensor intakeGripColor;

    //Initialization of GRIP_STATE
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN(0.25),
        CLOSED(0.75);

        private final double gripPosition;
        GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public GRIP_STATE gripState = IntakeArm.GRIP_STATE.CLOSED;

    public enum ARM_STATE{
        AUTO_CONE_1(0.6,1),
        AUTO_CONE_2(0.7,2),
        AUTO_CONE_3(0.8,3),
        AUTO_CONE_4(0.9, 4),
        AUTO_CONE_5(1,5),
        PICKUP(1,6),
        PICKUP_WRIST_DOWN_POSITION(0.7,7),
        RANDOM(0,8),
        TRANSFER(0.0,9);

        private final double motorPosition;
        private final double index;
        ARM_STATE(double motorPosition, int index){
            this.motorPosition = motorPosition;
            this.index = index;
        }
        private ARM_STATE armStateByIndex(int index) {
            return ARM_STATE.values()[index-1];
        }

    }
    public ARM_STATE armState = ARM_STATE.TRANSFER;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        UP(0.2),
        LEVEL(0.5),
        TRANSFER (0.7),
        DOWN(1);

        private final double wristPosition;
        WRIST_STATE(double wristPosition) {
            this.wristPosition = wristPosition;
        }
    }

    public enum INTAKE_GRIP_COLOR_SENSOR_STATE {
        DETECTED,
        NOT_DETECTED
    }

    public INTAKE_GRIP_COLOR_SENSOR_STATE intakeGripColorSensorState = INTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;
    public WRIST_STATE wristState = WRIST_STATE.TRANSFER;

    public IntakeArm(HardwareMap hardwareMap) { //map hand servo's to each
        intakeArmServo = hardwareMap.get(Servo.class, "intakeArmServo");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");
        intakeGripServo = hardwareMap.get(Servo.class, "intakeGripServo");

        intakeGripColor = hardwareMap.get(NormalizedColorSensor.class, "intake_grip_sensor");
        initIntakeArm();
    }

    //initialize intakeArm
    public void initIntakeArm() {
        if (intakeGripColor instanceof SwitchableLight) {
            ((SwitchableLight)intakeGripColor).enableLight(true);
        }
        closeGrip();
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        intakeGripServo.setPosition(GRIP_STATE.OPEN.gripPosition);
        gripState = GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        intakeGripServo.setPosition(GRIP_STATE.CLOSED.gripPosition);
        gripState = GRIP_STATE.CLOSED;
    }

    public void toggleGrip(){
        if (gripState == GRIP_STATE.CLOSED) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    public static final double WRIST_UP_DELTA = 0.2;

    public void moveWrist(WRIST_STATE toWristState) {
        switch (toWristState) {
            case LEVEL:
                intakeWristServo.setPosition(determineWristLevelPosition(/*TODO*/));
                break;
            case UP:
                intakeWristServo.setPosition(determineWristLevelPosition(/*TODO*/) + WRIST_UP_DELTA);
                break;
            case TRANSFER:
            case DOWN:
                intakeWristServo.setPosition(toWristState.wristPosition);
                break;
        }
        wristState = toWristState;
    }

    public void moveArmWristUpOneStack(){

    }

    public void moveArmWristFallenCone(){

    }

    public void moveArmWristToPickUp(){

    }

    public void continousArmRotateUp(){

    }

    public void continousArmRotateDown(){

    }

    //Algorithm to determine wrist position based on arm angle
    public double determineWristLevelPosition(/*double armAngle TODO*/){
        double determinedWristPosition = 0;
        /*
        wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION + ((armAngle - SystemState.SHOULDER_PICKUP_POSITION)
                / SystemState.SHOULDER_WRIST_ANGLE_FACTOR);
        wristUpPosition = wristLevelPosition + WRIST_DELTA_FOR_HIGH;
         */
        return determinedWristPosition;
    }

    public double intakeGripDistance;
    /**
     * Returns the color sensor state back, and sets specific values to check if the sensor
     * is detecting anything
     * @return
     */
    public GRIP_STATE getIntakeGripColorDistanceSensorState(){
        if (intakeGripColor instanceof DistanceSensor) {
            intakeGripDistance =  ((DistanceSensor) intakeGripColor).getDistance(DistanceUnit.CM);
        }

        if (intakeGripDistance < 4) {
            gripState = GRIP_STATE.CLOSED;
        } else {
            gripState = GRIP_STATE.OPEN;
        }
        return gripState;
    }

    public double getIntakeGripColorSensorDistance(){
        return intakeGripDistance;
    }


}
