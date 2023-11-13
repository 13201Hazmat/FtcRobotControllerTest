package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Magazine {
    public NormalizedColorSensor magazineSensorBottom;
    public NormalizedColorSensor magazineSensorTop;
    public Servo magazineDoorServo;

    public enum MAGAZINE_DOOR_STATE {
        CLOSED(0.0),
        OPEN(0.0);

        private double magazinePosition;

        MAGAZINE_DOOR_STATE(double magazinePosition){ this.magazinePosition = magazinePosition; }
        public double getMagazinePosition(){ return magazinePosition; }
    }
    public MAGAZINE_DOOR_STATE magazineDoorState = MAGAZINE_DOOR_STATE.CLOSED;

    public Telemetry telemetry;

    public enum MAGAZINE_STATE {
        EMPTY,
        LOADED_ONE_PIXEL,
        LOADED_TWO_PIXEL
    }
    public MAGAZINE_STATE magazineState = MAGAZINE_STATE.EMPTY;

    public double MAGAZINE_SENSE_DISTANCE = 100;

    public Magazine(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        magazineDoorServo = hardwareMap.get(Servo.class, "magazine_servo");
        magazineSensorBottom = hardwareMap.get(NormalizedColorSensor.class, "magazine_bottom");
        magazineSensorTop = hardwareMap.get(NormalizedColorSensor.class, "magazine_top");

        initMagazine();
    }

    public void initMagazine(){
        closeMagazineDoor();
        senseMagazineState();
    }

    /**
     * Moving the box wall down
     */
    //UPDATE FOR THIS YEAR!!
    public void openMagazineDoor(){
        magazineDoorServo.setPosition(MAGAZINE_DOOR_STATE.OPEN.magazinePosition);
        magazineDoorState = MAGAZINE_DOOR_STATE.OPEN;
    }

    /**
     * Moving the box wall up
     */
    //UPDATE FOR THIS YEAR!!
    public void closeMagazineDoor(){
        magazineDoorServo.setPosition(MAGAZINE_DOOR_STATE.CLOSED.magazinePosition);
        magazineDoorState = MAGAZINE_DOOR_STATE.CLOSED;
    }

    public double magazineDistanceBottom;
    public double magazineDistanceTop;
    public void senseMagazineState(){
        boolean magazinePixelBottomSensed = false;
        boolean magazinePixelTopSensed = false;

        if (magazineSensorBottom instanceof DistanceSensor){
            magazineDistanceBottom = ((DistanceSensor) magazineSensorBottom).getDistance(DistanceUnit.MM);
        }
        if(magazineDistanceBottom < MAGAZINE_SENSE_DISTANCE){
            magazinePixelBottomSensed = true;
        }

        if (magazineSensorTop instanceof DistanceSensor) {
            magazineDistanceTop = ((DistanceSensor) magazineSensorTop).getDistance(DistanceUnit.MM);
        }

        if (magazineDistanceTop < MAGAZINE_SENSE_DISTANCE) {
            magazinePixelTopSensed = true;
        }

        if (!magazinePixelBottomSensed) {
            magazineState = MAGAZINE_STATE.EMPTY;
        } else { // magazinePixelBottomSensed == true
            if (!magazinePixelTopSensed) {
                magazineState = MAGAZINE_STATE.LOADED_ONE_PIXEL;
            } else { //magazinePixelTopSensed == true
                magazineState = MAGAZINE_STATE.LOADED_TWO_PIXEL;
            }
        }

    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Magazinee");
        telemetry.addData("    State", magazineState);
        telemetry.addData("    Bottom Distance Sensed", magazineDistanceBottom);
        telemetry.addData("    Top Distance Sensed", magazineDistanceTop);
        telemetry.addData("    Door State", magazineDoorState);
        telemetry.addData("    Door Servo Position", magazineDoorServo.getPosition());
        telemetry.addLine("=============");
    }

}