package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ParkingArm {
    public Servo parkingArmServo;
    public boolean deploy = false;

    public enum PARKING_ARM_SERVO_STATE {
        PARKING_ARM_RETRACTED(0.97), //TODO : Update Value
        PARKING_ARM_EXTENDED(0.37); //TODO : Update Value

        private double parkingArmPosition;

        PARKING_ARM_SERVO_STATE(double moveParkingArmPosition){
            this.parkingArmPosition = moveParkingArmPosition;
        }

        public double getParkingArmPosition(){return parkingArmPosition;}
    }
    public PARKING_ARM_SERVO_STATE parkingArmServoState = PARKING_ARM_SERVO_STATE.PARKING_ARM_RETRACTED;

    public Telemetry telemetry;
    public ParkingArm(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        parkingArmServo = hardwareMap.get(Servo.class, "parking_arm_servo");
        initParkingArm();
    }

    public void initParkingArm(){
        parkingArmServo.setPosition(PARKING_ARM_SERVO_STATE.PARKING_ARM_RETRACTED.getParkingArmPosition());
        parkingArmServoState = PARKING_ARM_SERVO_STATE.PARKING_ARM_RETRACTED;
    }

    public void extendParkingArm(){
        if (deploy) {
            parkingArmServo.setPosition(PARKING_ARM_SERVO_STATE.PARKING_ARM_EXTENDED.getParkingArmPosition());
            parkingArmServoState = PARKING_ARM_SERVO_STATE.PARKING_ARM_EXTENDED;
            deploy = false;
        }
    }

    public Action extendParkingArmAction(){
        return new Action(){
            @Override
            public void preview(Canvas canvas){}
            @Override
            public boolean run(TelemetryPacket packet){
                extendParkingArm();
                return false;
            }
        };
    }

    public void printDebugMessages(){
        //******  debug ******
        telemetry.addLine("ParkingArm");
        telemetry.addData("    Servo state", parkingArmServoState);
        telemetry.addData("    Servo position", parkingArmServo.getPosition());
        telemetry.addLine("=============");
    }
}
