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

    public NormalizedColorSensor outtakeWristColor;
    public NormalizedColorSensor outtakeGripColor;

    //Initialization of GRIP_STATE
    public enum GRIP_STATE { //state of the Hand Grip
        OPEN,
        CLOSED
    }
    public GRIP_STATE gripState = GRIP_STATE.CLOSED;
    //constants for Hand and grip position
    public static final double OPEN_GRIP_POS = 0.45; //value of Grip to open
    public static final double CLOSE_GRIP_POS = 0; //value of Grip to close
    //public static final double CLOSE_GRIP_FULL_POSITION = 0.00;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        WRIST_TRANSFER,
        WRIST_DROP
    }
    public WRIST_STATE wristState = WRIST_STATE.WRIST_TRANSFER;
    public static final double WRIST_TRANSFER_POSITION = 0.50;
    public double WRIST_DOWN_POSITION = 0.06;

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
        if (GameField.opModeRunning == HAZMAT_AUTONOMOUS) {
            closeGrip();
        }
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
        gripState = GRIP_STATE.CLOSED;
    }

    public void toggleGrip(){
        if (gripState == GRIP_STATE.CLOSED) {
            openGrip();
        } else {
            closeGrip();
        }
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

            if (outtakeGripDistance < 20) {
                outtakeConeSensed = true;
            } else {
                outtakeConeSensed = false;
            }
        }
        return outtakeConeSensed;
    }

    //rotates hand up given controller input
    public void moveWristTransfer(){
        //determineWristLevelPosition(shoulderLevelPosition);
        outtakeWristServo.setPosition(WRIST_TRANSFER_POSITION);
        wristState = WRIST_STATE.WRIST_TRANSFER;
    }

    //rotates hand down given controller input
    public void moveWristDrop(){
        outtakeWristServo.setPosition(WRIST_DOWN_POSITION);
        wristState = WRIST_STATE.WRIST_DROP;
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
            wristState = WRIST_STATE.WRIST_DROP;
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
