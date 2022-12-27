package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class IntakeArm {
    public Servo intakeArmServoLeft, intakeArmServoRight;
    public Servo intakeWristServoLeft, intakeWristServoRight;
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
        PICKUP(1, 0,0),
        AUTO_CONE_1(1,0,1),
        AUTO_CONE_2(0.9, 0.1, 2),
        AUTO_CONE_3(0.8, 0.2, 3),
        AUTO_CONE_4(0.7, 0.3, 4),
        AUTO_CONE_5(0.6, 0.4, 5),

        PICKUP_FALLEN_CONE(0.7, 0.3,  6),
        RANDOM(0,0,7),
        TRANSFER(0.6,0.4,8);

        private final double leftArmPosition, rightArmPosition;
        private final int index;
        ARM_STATE(double leftArmPosition, double rightArmPosition, int index){
            this.leftArmPosition = leftArmPosition;
            this.rightArmPosition = rightArmPosition;
            this.index = index;
        }

        private ARM_STATE byIndex(int ord) {
            if (ord <1) ord = 1;
            if (ord >5) ord = 5;
            for (ARM_STATE a : ARM_STATE.values()) {
                if (a.index == ord) {
                    return a;
                }
            }
            return null;
        }
    }
    public ARM_STATE armState = ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        UP(0.2, 0.8),
        PICKUP_LEVEL(0.5, 0.5),
        AUTO_CONE_5(1, 0),
        TRANSFER (0.7,0.3),
        FALLEN_CONE(1,0);

        private final double leftWristPosition;
        private final double rightWristPosition;
        WRIST_STATE(double leftWristPosition, double rightWristPosition) {
            this.leftWristPosition = leftWristPosition;
            this.rightWristPosition = rightWristPosition;
        }
    }
    public WRIST_STATE wristState = WRIST_STATE.TRANSFER;

    public enum INTAKE_GRIP_COLOR_SENSOR_STATE {
        DETECTED,
        NOT_DETECTED
    }

    public INTAKE_GRIP_COLOR_SENSOR_STATE intakeGripColorSensorState = INTAKE_GRIP_COLOR_SENSOR_STATE.NOT_DETECTED;

    public IntakeArm(HardwareMap hardwareMap) { //map hand servo's to each
        intakeArmServoLeft = hardwareMap.get(Servo.class, "intake_arm_left");
        intakeArmServoRight = hardwareMap.get(Servo.class, "intake_arm_right");
        intakeWristServoLeft = hardwareMap.get(Servo.class, "intake_wrist_left");
        intakeWristServoRight = hardwareMap.get(Servo.class, "intake_wrist_right");
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

    public void moveArm(ARM_STATE toArmState) {
        intakeArmServoLeft.setPosition(toArmState.leftArmPosition);
        intakeArmServoRight.setPosition(toArmState.rightArmPosition);
        armState = toArmState;
    }

    public void moveArmWristUpOneStack(){
        if (armState.index < 1 || armState.index > 4) {
            return;
        } else {
            assert armState.byIndex(armState.index + 1) != null;
            moveArm(armState.byIndex(armState.index + 1));
        }
    }

    public void continousArmRotateUp(){
        intakeArmServoLeft.setPosition(intakeArmServoLeft.getPosition() + ARM_DELTA);
        intakeArmServoRight.setPosition(intakeArmServoRight.getPosition() - ARM_DELTA);
        armState = ARM_STATE.RANDOM;
    }

    public void continousArmRotateDown(){
        intakeArmServoLeft.setPosition(intakeArmServoLeft.getPosition() - ARM_DELTA);
        intakeArmServoRight.setPosition(intakeArmServoRight.getPosition() + ARM_DELTA);
        armState = ARM_STATE.RANDOM;
    }

    public static final double WRIST_UP_DELTA = 0.2;

    public void moveWrist(WRIST_STATE toWristState) {
        switch (toWristState) {
            case PICKUP_LEVEL:
            case AUTO_CONE_5:
                intakeWristServoLeft.setPosition(determineWristLevelLeft(/*TODO*/));
                intakeWristServoRight.setPosition(determineWristLevelRight(/*TODO*/));
                break;
            case UP:
                intakeWristServoLeft.setPosition(determineWristLevelLeft(/*TODO*/) + WRIST_UP_DELTA);
                intakeWristServoRight.setPosition(determineWristLevelRight(/*TODO*/) + WRIST_UP_DELTA);
                break;
            case TRANSFER:
            case FALLEN_CONE:
                intakeWristServoLeft.setPosition(toWristState.leftWristPosition);
                intakeWristServoRight.setPosition(toWristState.leftWristPosition);
                break;
        }
        wristState = toWristState;
    }

    //Algorithm to determine wrist position based on arm angle
    public double determineWristLevelLeft(/*double armAngle TODO*/){
        double determinedWristLeft = 0;
        /*
        wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION + ((armAngle - SystemState.SHOULDER_PICKUP_POSITION)
                / SystemState.SHOULDER_WRIST_ANGLE_FACTOR);
        wristUpPosition = wristLevelPosition + WRIST_DELTA_FOR_HIGH;
         */
        return determinedWristLeft;
    }

    public double determineWristLevelRight(/*double armAngle TODO*/){
        double determinedWristRight = 0;
        /*
        wristLevelPosition = WRIST_PICKUP_LEVEL_POSITION + ((armAngle - SystemState.SHOULDER_PICKUP_POSITION)
                / SystemState.SHOULDER_WRIST_ANGLE_FACTOR);
        wristUpPosition = wristLevelPosition + WRIST_DELTA_FOR_HIGH;
         */
        return determinedWristRight;
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

    public double getIntakeGripColorSensorDistance(){
        return intakeGripDistance;
    }


}
