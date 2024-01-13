package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class VisionSensor {

    public Telemetry telemetry;
    public DistanceSensor backdropDistanceSensorLeft, backdropDistanceSensorRight;
    public HuskyLens huskyLens;
    public final int READ_PERIOD = 1;

    public enum BACKDROP_DISTANCE_STATE {
        NOT_SENSED,
        AMBER,
        RED
    }
    public BACKDROP_DISTANCE_STATE backdropDistanceState = BACKDROP_DISTANCE_STATE.NOT_SENSED;

    public enum BACKDROP_DISTANCE_SENSOR_STATE {
        NOT_SENSED,
        AMBER,
        RED
    }
    BACKDROP_DISTANCE_SENSOR_STATE backdropDistanceLeftState = BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED;
    BACKDROP_DISTANCE_SENSOR_STATE backdropDistanceRightState = BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED;

    public String[] backdropAprilTags = new String[]{
                    "a",
                    "b",
                    "c",
                    "d",
                    "e",
                    "f"
            };

    public boolean backdropAprilTagDetected = false;
    public String detectedAprilTag = "";
    public int detectedAprilTagCount = 0;

    public double DISTANCE_SENSOR_THRESHOLD_AMBER = 150; //mm
    public double DISTANCE_SENSOR_THRESHOLD_RED = 75; //mm

    public double backdropDistanceLeftDistance = 0;
    public double backdropDistanceRightDistance = 0;

    public VisionSensor(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        //backdropDistanceSensorLeft = hardwareMap.get(DistanceSensor.class, "back_distance_left");
        //backdropDistanceSensorRight = hardwareMap.get(DistanceSensor.class, "back_distance_right");

        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

    }


    public void senseBackdrop(){/*
        backdropDistanceLeftDistance = backdropDistanceSensorLeft.getDistance(DistanceUnit.MM);
        backdropDistanceRightDistance = backdropDistanceSensorRight.getDistance(DistanceUnit.MM);

        if (senseAprilTag()) {
            if (backdropDistanceLeftDistance < DISTANCE_SENSOR_THRESHOLD_AMBER) {
                backdropDistanceLeftState = BACKDROP_DISTANCE_SENSOR_STATE.AMBER;
            } else if (backdropDistanceLeftDistance < DISTANCE_SENSOR_THRESHOLD_RED) {
                backdropDistanceLeftState = BACKDROP_DISTANCE_SENSOR_STATE.RED;
            } else {
                backdropDistanceLeftState = BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED;
            }

            if (backdropDistanceRightDistance < DISTANCE_SENSOR_THRESHOLD_AMBER) {
                backdropDistanceRightState = BACKDROP_DISTANCE_SENSOR_STATE.AMBER;
            } else if (backdropDistanceRightDistance < DISTANCE_SENSOR_THRESHOLD_RED) {
                backdropDistanceRightState = BACKDROP_DISTANCE_SENSOR_STATE.RED;
            } else {
                backdropDistanceRightState = BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED;
            }

            if (backdropDistanceLeftState == BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED ||
                    backdropDistanceRightState == BACKDROP_DISTANCE_SENSOR_STATE.NOT_SENSED) {
                backdropDistanceState = BACKDROP_DISTANCE_STATE.NOT_SENSED;
            }

            if (backdropDistanceLeftState == BACKDROP_DISTANCE_SENSOR_STATE.AMBER ||
                    backdropDistanceRightState == BACKDROP_DISTANCE_SENSOR_STATE.AMBER) {
                backdropDistanceState = BACKDROP_DISTANCE_STATE.AMBER;
            }

            if (backdropDistanceLeftState == BACKDROP_DISTANCE_SENSOR_STATE.RED ||
                    backdropDistanceRightState == BACKDROP_DISTANCE_SENSOR_STATE.RED) {
                backdropDistanceState = BACKDROP_DISTANCE_STATE.RED;
            }
        } else {
            backdropDistanceState = BACKDROP_DISTANCE_STATE.NOT_SENSED;
        }
*/
    }

    String detectedAprilTagName = "";

    public boolean senseAprilTag(){
        backdropAprilTagDetected = true;

        /*HuskyLens.Block[] blocks = huskyLens.blocks();
        detectedAprilTagCount = blocks.length;
        if (detectedAprilTagCount > 0)
            backdropAprilTagDetected = true;

         */

        /*
        backdropAprilTagDetected = false;
        for (int i = 0; i < detectedAprilTagCount; i++) {
            for (int j = 0; (j < backdropAprilTags.length & !backdropAprilTagDetected); j++) {
                if (blocks[i].toString().equals(backdropAprilTags[j])) {
                    backdropAprilTagDetected = true;
                    detectedAprilTag = backdropAprilTags[j];
                }
            }
        }*/

        return backdropAprilTagDetected;
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("VisionSensor");
        telemetry.addData("    backdropDistanceState", backdropDistanceState);
        telemetry.addData("    backdrop Left Distance", backdropDistanceLeftDistance);
        telemetry.addData("    backdrop Right Distance", backdropDistanceRightDistance);
        telemetry.addData("    HuskyLens detected AprilTag Count", detectedAprilTagCount);
        //telemetry.addData("    HuskyLens detected AprilTag", detectedAprilTag.toString());
        telemetry.addLine("=============");
    }

}
