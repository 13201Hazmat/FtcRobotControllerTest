package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeArm {
    public Servo intakeArmServo;
    public Servo intakeWristServo;
    public Servo intakeGripServo;

    public NormalizedColorSensor intakeGripColor;

    //Initialization of GRIP_STATE
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN,
        CLOSE
    }
    public IntakeArm.GRIP_STATE gripState = IntakeArm.GRIP_STATE.CLOSE;

    public enum ARM_STATE{
        CONE_1(0),
        CONE_2(0),
        CONE_3(0),
        CONE_4(0),
        CONE_5(0),
        PICKUP_WRIST_DOWN_POSITION(0),
        RANDOM(0);

        private final double motorPosition;
        ARM_STATE(double motorPosition){this.motorPosition = motorPosition;}
    }
    public ARM_STATE armState = ARM_STATE.CONE_5;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        WRIST_TRANSFER,
        WRIST_LEVEL,
        WRIST_UP,
        WRIST_DOWN
    }

    public enum INTAKE_GRIP_COLOR_SENSOR_STATE {
        DETECTED,
        NOT_DETECTED
    }

    public INTAKE_GRIP_COLOR_SENSOR_STATE intakeGripColorSensorState = INTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;
    public WRIST_STATE wristState = WRIST_STATE.WRIST_TRANSFER;

    //constants for Hand and grip position
    public static final double OPEN_GRIP_POS = 0.45; //value of Grip to open
    public static final double CLOSE_GRIP_POS = 0; //value of Grip to close
    //public static final double CLOSE_GRIP_FULL_POSITION = 0.00;

    public static final double WRIST_TRANSFER_POSITION = 0.50;
    public static final double WRIST_UP_POSITION = 0.06;
    public double WRIST_LEVEL_POSITION = 0.01; //Will dynamically change based on arm angle


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
        intakeGripServo.setPosition(OPEN_GRIP_POS);
        gripState = GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        intakeGripServo.setPosition(CLOSE_GRIP_POS);
        gripState = GRIP_STATE.CLOSE;
    }

    public void toggleGrip(){
        if (gripState == GRIP_STATE.CLOSE) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    public void moveWristTransfer(){
        //determineWristLevelPosition(shoulderLevelPosition);
        intakeWristServo.setPosition(WRIST_TRANSFER_POSITION);
        wristState = WRIST_STATE.WRIST_TRANSFER;
    }

    //rotates hand down given controller input
    public void moveWristUp(){
        intakeWristServo.setPosition(WRIST_UP_POSITION);
        wristState = WRIST_STATE.WRIST_UP;
    }

    public void moveWristLevel(double armAngle){
        determineWristLevelPosition(armAngle);
        intakeWristServo.setPosition(WRIST_LEVEL_POSITION);
        wristState = WRIST_STATE.WRIST_LEVEL;
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
    public void determineWristLevelPosition(double armAngle){
        /*
        wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION + ((armAngle - SystemState.SHOULDER_PICKUP_POSITION)
                / SystemState.SHOULDER_WRIST_ANGLE_FACTOR);
        wristUpPosition = wristLevelPosition + WRIST_DELTA_FOR_HIGH;
         */
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
            gripState = GRIP_STATE.CLOSE;
        } else {
            gripState = GRIP_STATE.OPEN;
        }
        return gripState;
    }

    public double getIntakeGripColorSensorDistance(){
        return intakeGripDistance;
    }


}
