package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class OuttakeArm {
    //Initialization of <outtake arm servo's>
    public Servo outtakeWristServo;
    public Servo outtakeGripServo;

    public NormalizedColorSensor outtakeWristColor;
    public NormalizedColorSensor outtakeGripColor;

    //Initialization of GRIP_STATE
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN,
        CLOSE
    }
    public GRIP_STATE gripState = GRIP_STATE.CLOSE;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        WRIST_TRANSFER,
        WRIST_DOWN
    }

    public enum OUTTAKE_GRIP_COLOR_SENSOR_STATE {
        DETECTED,
        NOT_DETECTED
    }

    public OUTTAKE_GRIP_COLOR_SENSOR_STATE outtakeGripColorSensorState = OUTTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;
    public WRIST_STATE wristState = WRIST_STATE.WRIST_TRANSFER;

    //constants for Hand and grip position
    public static final double OPEN_GRIP_POS = 0.45; //value of Grip to open
    public static final double CLOSE_GRIP_POS = 0; //value of Grip to close
    //public static final double CLOSE_GRIP_FULL_POSITION = 0.00;

    public static final double WRIST_TRANSFER_POSITION = 0.50;
    public double WRIST_DOWN_POSITION = 0.06;

    public OuttakeArm(HardwareMap hardwareMap) { //map hand servo's to each
        outtakeWristServo = hardwareMap.get(Servo.class, "outtakeWristServo");
        outtakeGripServo = hardwareMap.get(Servo.class, "outtakeGripServo");

        outtakeWristColor = hardwareMap.get(NormalizedColorSensor.class, "outtake_wrist_sensor");
        outtakeGripColor = hardwareMap.get(NormalizedColorSensor.class, "outtake_grip_sensor");
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
        closeGrip();
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        outtakeGripServo.setPosition(OPEN_GRIP_POS);
        gripState = GRIP_STATE.OPEN;
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        outtakeGripServo.setPosition(CLOSE_GRIP_POS);
        gripState = GRIP_STATE.CLOSE;
    }

    public void toggleGrip(){
        if (gripState == GRIP_STATE.CLOSE) {
            openGrip();
        } else {
            closeGrip();
        }
    }

    //rotates hand up given controller input
    public void moveWristTransfer(){
        //determineWristLevelPosition(shoulderLevelPosition);
        outtakeWristServo.setPosition(WRIST_TRANSFER_POSITION);
        wristState = WRIST_STATE.WRIST_TRANSFER;
    }

    //rotates hand down given controller input
    public void moveWristDown(){
        outtakeWristServo.setPosition(WRIST_DOWN_POSITION);
        wristState = WRIST_STATE.WRIST_DOWN;
    }

    public double outtakeWristDistance;
    /**
     * Returns the color sensor state back, and sets specific values to check if the sensor
     * is detecting anything
     * @return
     */
    public WRIST_STATE getOuttakeWristColorDistanceSensorState(){
        if (outtakeWristColor instanceof DistanceSensor) {
            outtakeWristDistance =  ((DistanceSensor) outtakeWristColor).getDistance(DistanceUnit.CM);
        }

        if (outtakeWristDistance < 4) {
            wristState = WRIST_STATE.WRIST_DOWN;
        } else {
            wristState = WRIST_STATE.WRIST_TRANSFER;
        }
        return wristState;
    }

    public double getOuttakeWristColorSensorDistance(){
        return outtakeWristDistance;
    }

    //TODO:How to detect if a cone is in transfer pos with only a color sensor on grip
    public OUTTAKE_GRIP_COLOR_SENSOR_STATE getOuttakeGripColorSensorState(){

        /*
        if (outtakeWristDistance < 4) {
            outtakeGripColorSensorState = OUTTAKE_GRIP_COLOR_SENSOR_STATE.DETECTED;
        } else {
            outtakeGripColorSensorState = OUTTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;
        }

         */
        return outtakeGripColorSensorState;
    }


}
