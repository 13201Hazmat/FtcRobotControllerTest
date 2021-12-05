package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Definition of Magazine Class <BR>
 *
 * Example : Magazine consists of system provided magazine controls and adds functionality to the selection made on magazine. <BR>
 *
 * The states are as followed: <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing </emsp> <BR>
 *     <emsp>SUBSYSTEM1_SERVO_LEVEL2 for another state  = example if the intake is on or off </emsp> <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     <emsp>runSubsystem1Motor checks if the motor is not running and runs the intake </emsp> <BR>
 *     <emsp>stopSubsystem1Motor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets subsystem1MotorState to SUBSYSTEM1_SERVO_LEVEL1.STOPPED </emsp> <BR>
 *     <emsp> startReverseSubsystem1Motor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING</emsp> <BR>
 */
public class Magazine {

    //TODO: Update code as needed for Magazine

    public Servo magazineServo = null;
    public NormalizedColorSensor magazineColorSensor;

    public static final double MAGAZINE_SERVO_COLLECT_POSITION = 0.65;
    public static final double MAGAZINE_SERVO_TRANSPORT_POSITION =  0.55;
    public static final double MAGAZINE_SERVO_DROP_POSITION =  0.2;

    public enum MAGAZINE_SERVO_STATE {
        COLLECT,
        TRANSPORT,
        DROP
    }

    public enum MAGAZINE_BUTTON_STATE {
        ON,
        OFF
    }

    public MAGAZINE_BUTTON_STATE magazineButtonState;
    public MAGAZINE_SERVO_STATE magazineServoState = MAGAZINE_SERVO_STATE.TRANSPORT;

    public enum MAGAZINE_COLOR_SENSOR_STATE {
        EMPTY,
        LOADED
    }
    public MAGAZINE_COLOR_SENSOR_STATE magazineColorSensorState = MAGAZINE_COLOR_SENSOR_STATE.EMPTY;


    public Magazine(HardwareMap hardwareMap) {
        magazineServo = hardwareMap.servo.get("magazine_servo");
        magazineColorSensor = hardwareMap.get(NormalizedColorSensor.class, "magazine_sensor");
        initMagazine();
    }

    public void initMagazine(){
        moveMagazineToCollect();
    }

    /**
     * Sets magazineServo to collect position
     */
    public void moveMagazineToCollect(){
        if (magazineServoState != MAGAZINE_SERVO_STATE.COLLECT) {
            magazineServo.setPosition(MAGAZINE_SERVO_COLLECT_POSITION);
            magazineServoState = MAGAZINE_SERVO_STATE.COLLECT;
        }
    }

    /**
     * sets magazineServo to transport position
     */
    public void moveMagazineToTransport(){
        if (magazineServoState != MAGAZINE_SERVO_STATE.TRANSPORT) {
            magazineServo.setPosition(MAGAZINE_SERVO_TRANSPORT_POSITION);
            magazineServoState = MAGAZINE_SERVO_STATE.TRANSPORT;
        }
    }

    /**
     * sets magazineServo to flipped position
     */
    public void moveMagazineToDrop(){
        if (magazineServoState != MAGAZINE_SERVO_STATE.DROP) {
            magazineServo.setPosition(MAGAZINE_SERVO_DROP_POSITION);
            magazineServoState = MAGAZINE_SERVO_STATE.DROP;
        }
    }

    /**
     * Returns Magazine servo state
     * @return
     */
    public MAGAZINE_SERVO_STATE getMagazineServoState() {
        return magazineServoState;
    }

    public double magazineColorSensorDistance;

    public MAGAZINE_COLOR_SENSOR_STATE getMagazineColorSensorState(){
        if (magazineColorSensor instanceof DistanceSensor) {
            magazineColorSensorDistance =  ((DistanceSensor) magazineColorSensor).getDistance(DistanceUnit.CM);
        }

        if (magazineColorSensorDistance < 3.0) {
            magazineColorSensorState = MAGAZINE_COLOR_SENSOR_STATE.LOADED;
        } else {
            magazineColorSensorState = MAGAZINE_COLOR_SENSOR_STATE.EMPTY;
        }
        return magazineColorSensorState;
    }

    public double getMagazineColorSensorDistance (){
        return magazineColorSensorDistance;
    }

}