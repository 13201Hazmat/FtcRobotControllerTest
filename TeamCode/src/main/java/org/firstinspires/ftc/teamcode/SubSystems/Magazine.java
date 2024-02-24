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
    public boolean magazineSensorActivated = true;

    public Telemetry telemetry;

    public enum MAGAZINE_STATE {
        EMPTY,
        LOADED_ONE_PIXEL,
        LOADED_TWO_PIXEL
    }
    public MAGAZINE_STATE magazineState = MAGAZINE_STATE.EMPTY;
    public MAGAZINE_STATE magazinePreviousState = MAGAZINE_STATE.EMPTY;

    public double MAGAZINE_SENSE_DISTANCE = 9.0;

    public Magazine(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        //magazineDoorServo = hardwareMap.get(Servo.class, "magazine_servo");
        magazineSensorBottom = hardwareMap.get(NormalizedColorSensor.class, "magazine_bottom");
        magazineSensorTop = hardwareMap.get(NormalizedColorSensor.class, "magazine_top");

        initMagazine();
    }

    public void initMagazine(){
        senseMagazineState();
    }

    public double magazineDistanceBottom;
    public double magazineDistanceTop;
    public void senseMagazineState(){
        if (!magazineSensorActivated) {
            magazinePreviousState = magazineState;
            magazineState = MAGAZINE_STATE.EMPTY;
            return;
        }

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
            magazinePreviousState = magazineState;
            magazineState = MAGAZINE_STATE.EMPTY;
        } else { // magazinePixelBottomSensed == true
            if (!magazinePixelTopSensed) {
                magazinePreviousState = magazineState;
                magazineState = MAGAZINE_STATE.LOADED_ONE_PIXEL;
            } else { //magazinePixelTopSensed == true
                magazinePreviousState = magazineState;
                magazineState = MAGAZINE_STATE.LOADED_TWO_PIXEL;
            }
        }

    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addLine("Magazine");
        telemetry.addData("    SensorActivated", magazineSensorActivated);
        telemetry.addData("    State", magazineState);
        telemetry.addData("    Bottom Distance Sensed", magazineDistanceBottom);
        telemetry.addData("    Top Distance Sensed", magazineDistanceTop);
        telemetry.addLine("=============");
    }

}