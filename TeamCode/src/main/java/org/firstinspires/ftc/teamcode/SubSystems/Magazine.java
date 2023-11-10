package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class Magazine {
    public NormalizedColorSensor magazineSensor1;
    public NormalizedColorSensor magazineSensor2;
    public Servo magazineServo;

    public enum MAGAZINE_STATE {
        CLOSED(0.0),
        EJECTED(0.0);

        private double magazinePosition;

        MAGAZINE_STATE(double magazinePosition){ this.magazinePosition = magazinePosition; }
        public double getMagazinePosition(){ return magazinePosition; }
    }
    public MAGAZINE_STATE magazineState = MAGAZINE_STATE.CLOSED;

    public Telemetry telemetry;

    public enum TRANSFER_STATE {
        UP(0.0),
        EJECTED(0.0);

        private double transferPosition;

        TRANSFER_STATE(double transferPosition){ this.transferPosition = transferPosition; }
        public double getTransferPosition(){ return transferPosition; }
    }
    public TRANSFER_STATE transferState = TRANSFER_STATE.UP;

    public Magazine(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        magazineServo = hardwareMap.get(Servo.class, "box_servo");
        initMagazine();
    }

    public void initMagazine(){
        closeMagazine();
    }

    /**
     * Moving the box wall down
     */
    //UPDATE FOR THIS YEAR!!
    public void ejectPixel(){
        magazineServo.setPosition(MAGAZINE_STATE.EJECTED.magazinePosition);
        transferState = TRANSFER_STATE.EJECTED;
    }

    /**
     * Moving the box wall up
     */
    //UPDATE FOR THIS YEAR!!
    public void closeMagazine(){
        magazineServo.setPosition(MAGAZINE_STATE.CLOSED.magazinePosition);
        transferState = TRANSFER_STATE.UP;
    }

    public double magazineDistance1;
    public double magazineDistance2;
    public boolean senseMagazinePixel1(){
        boolean magazinePixelSensed = false;
        int senseDistance = 0;// need to test
        if (magazineSensor1 instanceof DistanceSensor){
            magazineDistance1 = ((DistanceSensor) magazineSensor1).getDistance(DistanceUnit.MM);
        }
        if(GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS){
            senseDistance = 10; //CHANGE
        } else {
            senseDistance = 10; //CHANGE
        }
        if(magazineDistance1 < senseDistance){
            magazinePixelSensed = true;
        } else {
            magazinePixelSensed = false;
        }
        return magazinePixelSensed;
    }

    public boolean senseMagazinePixel2(){
        boolean magazinePixelSensed = false;
        int senseDistance = 0;// need to test
        if (magazineSensor2 instanceof DistanceSensor){
            magazineDistance2 = ((DistanceSensor) magazineSensor2).getDistance(DistanceUnit.MM);
        }
        if(GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS){
            senseDistance = 10; //CHANGE
        } else {
            senseDistance = 10; //CHANGE
        }
        if(magazineDistance2 < senseDistance){
            magazinePixelSensed = true;
        } else {
            magazinePixelSensed = false;
        }
        return magazinePixelSensed;
    }

    public double getMagazineDistance1(){
        return magazineDistance1;
    }

    public double getMagazineDistance2(){
        return magazineDistance2;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);

        telemetry.addLine("=============");
    }

}