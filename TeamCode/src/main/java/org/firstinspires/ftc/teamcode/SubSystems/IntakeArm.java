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
    public enum INTAKE_GRIP_STATE { //state of the Hand Grip
        OPEN(0.65),
        OPEN_AUTO(0.65),
        CLOSED(1.0);

        private final double gripPosition;
        INTAKE_GRIP_STATE(double gripPosition) {
            this.gripPosition = gripPosition;
        }
    }
    public INTAKE_GRIP_STATE intakeGripState = INTAKE_GRIP_STATE.CLOSED;

    public boolean autoIntakeCloseMode = true;

    public enum INTAKE_ARM_STATE {
        INIT(0.49,0.50,0), //Level 15
        PICKUP_AUTO_CONE_1(0.01,0.95,1),
        AUTO_CONE_2(0.06, 0.90, 2),
        AUTO_CONE_3(0.10, 0.86, 3),
        AUTO_CONE_4(0.14, 0.82, 4),
        AUTO_CONE_5(0.19, 0.77, 5),
        TRANSFER(0.65,0.35,6), //Level 20
        LOW_JUNCTION(0.39,0.59,7), //Level 12
        PICKUP_FALLEN_CONE(0.17, 0.8,  8),
        RANDOM(0,0,9),
        RANDOM_MAX(0.46,0.53,10); //Level 14

        private double leftArmPosition;
        private double rightArmPosition;
        private final int index;
        INTAKE_ARM_STATE(double leftArmPosition, double rightArmPosition, int index){
            this.leftArmPosition = leftArmPosition;
            this.rightArmPosition = rightArmPosition;
            this.index = index;
        }

        public INTAKE_ARM_STATE byIndex(int ord) {
            for (INTAKE_ARM_STATE a : INTAKE_ARM_STATE.values()) {
                if (ord <1) ord = 1;
                if (ord >5) ord = 5;
                if (a.index == ord) {
                    return a;
                }
            }
            return null;
        }
    }
    public INTAKE_ARM_STATE intakeArmState = INTAKE_ARM_STATE.TRANSFER;
    public double ARM_DELTA = 0.01;

    //Hand - wrist, grip state declaration
    public enum WRIST_STATE {
        INIT(0.33,0.73),
        PICKUP_AUTO_CONE_1_LEVEL(0.26, 0.8),
        /*
        AUTO_CONE_2(0.23 ,0.77),
        AUTO_CONE_3(0.26, 0.74),
        AUTO_CONE_4(0.31, 0.69),
         */
        RANDOM (0.6,0.4),
        AUTO_CONE_5(0.40, 0.66),
        LOW_JUNCTION(0.4,0.6),
        TRANSFER (0.33,0.73),
        FALLEN_CONE(0.5,0.5);

        private final double leftWristPosition;
        private final double rightWristPosition;
        WRIST_STATE(double leftWristPosition, double rightWristPosition) {
            this.leftWristPosition = leftWristPosition;
            this.rightWristPosition = rightWristPosition;
        }
    }
    public WRIST_STATE wristState = WRIST_STATE.TRANSFER;

    public double WRIST_ARM_FACTOR_LEFT, WRIST_ARM_FACTOR_RIGHT = 1;

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
        intakeGripServo = hardwareMap.get(Servo.class, "intake_grip_servo");

        intakeGripColor = hardwareMap.get(NormalizedColorSensor.class, "intake_grip_sensor");
        initIntakeArm();
    }



    //initialize intakeArm
    public void initIntakeArm() {
        if (intakeGripColor instanceof SwitchableLight) {
            ((SwitchableLight)intakeGripColor).enableLight(true);
        }

        WRIST_ARM_FACTOR_LEFT = (WRIST_STATE.AUTO_CONE_5.leftWristPosition - WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.leftWristPosition)/
                (INTAKE_ARM_STATE.AUTO_CONE_5.leftArmPosition - INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1.leftArmPosition) ;

        WRIST_ARM_FACTOR_RIGHT = (WRIST_STATE.AUTO_CONE_5.rightWristPosition - WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.rightWristPosition)/
                (INTAKE_ARM_STATE.AUTO_CONE_5.rightArmPosition - INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1.rightArmPosition) ;

        moveArm(INTAKE_ARM_STATE.INIT);
        closeGrip();
    }

    public void moveArm(INTAKE_ARM_STATE toArmState) {
        intakeArmServoLeft.setPosition(toArmState.leftArmPosition);
        intakeArmServoRight.setPosition(toArmState.rightArmPosition);
        intakeArmState = toArmState;
        moveWrist(toArmState);
    }

    public void moveArmWristUpOneStack(){
        if (intakeArmState.index < 1 || intakeArmState.index > 5) {
            return;
        } else {
            assert intakeArmState.byIndex(intakeArmState.index + 1) != null;
            moveArm(intakeArmState.byIndex(intakeArmState.index + 1));
        }
    }

    public void moveArmWristDownOneStack(){
        if (intakeArmState.index <= 1 || intakeArmState.index > 6) {
            return;
        } else {
            assert intakeArmState.byIndex(intakeArmState.index - 1) != null;
            moveArm(intakeArmState.byIndex(intakeArmState.index - 1));
        }
    }


    public void continousArmRotateUp(){
        double deltaArmIntakeLeft = intakeArmServoLeft.getPosition() + ARM_DELTA;
        double deltaArmIntakeRight = intakeArmServoRight.getPosition() - ARM_DELTA;
        if(deltaArmIntakeLeft > INTAKE_ARM_STATE.RANDOM_MAX.leftArmPosition){
            deltaArmIntakeLeft = INTAKE_ARM_STATE.RANDOM_MAX.leftArmPosition;
            deltaArmIntakeRight = INTAKE_ARM_STATE.RANDOM_MAX.rightArmPosition;
        }

        intakeArmServoLeft.setPosition(deltaArmIntakeLeft);
        intakeArmServoRight.setPosition(deltaArmIntakeRight);
        intakeArmState = INTAKE_ARM_STATE.RANDOM;
        moveWrist(INTAKE_ARM_STATE.RANDOM);
    }

    public void continousArmRotateDown(){
        intakeArmState = INTAKE_ARM_STATE.RANDOM;
        intakeArmState.leftArmPosition = intakeArmServoLeft.getPosition() - ARM_DELTA;
        intakeArmState.rightArmPosition = intakeArmServoRight.getPosition() + ARM_DELTA;
        intakeArmServoLeft.setPosition(intakeArmState.leftArmPosition);
        intakeArmServoRight.setPosition(intakeArmState.rightArmPosition);
        intakeArmState = INTAKE_ARM_STATE.RANDOM;
        moveWrist(INTAKE_ARM_STATE.RANDOM);
    }

    public static final double WRIST_UP_DELTA = 0.2;

    public void moveWrist(INTAKE_ARM_STATE toArmState) {
        switch (toArmState) {
            case INIT:
                intakeWristServoLeft.setPosition(WRIST_STATE.INIT.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.INIT.rightWristPosition);
                wristState = WRIST_STATE.INIT;
                break;
            case PICKUP_AUTO_CONE_1:
                intakeWristServoLeft.setPosition(WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.rightWristPosition);
                wristState = WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL;
                break;
            case AUTO_CONE_2:
            case AUTO_CONE_3:
            case AUTO_CONE_4:
            case RANDOM:
                intakeWristServoLeft.setPosition(determineWristLevelLeft(intakeArmServoLeft.getPosition()));
                intakeWristServoRight.setPosition(determineWristLevelRight(intakeArmServoRight.getPosition()));
                wristState = WRIST_STATE.RANDOM;
                break;
            case AUTO_CONE_5:
                intakeWristServoLeft.setPosition(WRIST_STATE.AUTO_CONE_5.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.AUTO_CONE_5.rightWristPosition);
                wristState = WRIST_STATE.AUTO_CONE_5;
                break;
            case LOW_JUNCTION:
                intakeWristServoLeft.setPosition(WRIST_STATE.LOW_JUNCTION.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.LOW_JUNCTION.rightWristPosition);
                wristState = WRIST_STATE.LOW_JUNCTION;
                break;
            case TRANSFER:
                intakeWristServoLeft.setPosition(WRIST_STATE.TRANSFER.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.TRANSFER.rightWristPosition);
                wristState = WRIST_STATE.TRANSFER;
                break;
            case PICKUP_FALLEN_CONE:
                intakeWristServoLeft.setPosition(WRIST_STATE.FALLEN_CONE.leftWristPosition);
                intakeWristServoRight.setPosition(WRIST_STATE.FALLEN_CONE.rightWristPosition);
                wristState = WRIST_STATE.FALLEN_CONE;
                break;
        }
    }

    //Algorithm to determine wrist position based on arm angle
    public double determineWristLevelLeft(double leftArmPosition){
        return (WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.leftWristPosition +
                (leftArmPosition - INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1.leftArmPosition) * WRIST_ARM_FACTOR_LEFT);
    }

    public double determineWristLevelRight(double rightArmPosition){
        return (WRIST_STATE.PICKUP_AUTO_CONE_1_LEVEL.rightWristPosition +
                (rightArmPosition - INTAKE_ARM_STATE.PICKUP_AUTO_CONE_1.rightArmPosition) * WRIST_ARM_FACTOR_RIGHT);
    }

    public void moveWristUp(){
        if (wristState != WRIST_STATE.TRANSFER) {
            //TODO : CHECK WHY LOGIC IS NOT WORKING
            //intakeWristServoLeft.setPosition(determineWristLevelLeft(intakeWristServoLeft.getPosition()) - WRIST_UP_DELTA);
            //intakeWristServoRight.setPosition(determineWristLevelRight(intakeWristServoRight.getPosition()) + WRIST_UP_DELTA);
            intakeWristServoLeft.setPosition(intakeWristServoLeft.getPosition() - WRIST_UP_DELTA);
            intakeWristServoRight.setPosition(intakeWristServoRight.getPosition() + WRIST_UP_DELTA);
        }
    }

    public double intakeGripDistance;
    /**
     * Returns the color sensor state back, and sets specific values to check if the sensor
     * is detecting anything
     * @return
     */
    public boolean senseIntakeCone(){
        boolean intakeConeSensed = false;
        if (intakeArmState != INTAKE_ARM_STATE.TRANSFER) {
            if (intakeGripColor instanceof DistanceSensor) {
                intakeGripDistance = ((DistanceSensor) intakeGripColor).getDistance(DistanceUnit.MM);
            }

            if (intakeGripDistance < 40) {
                intakeConeSensed = true;
            } else {
                intakeConeSensed = false;
            }
        }
        return intakeConeSensed;
    }

    /**
     *If state of hand grip is set to open, set position of servo's to specified
     */
    public void openGrip(){
        intakeGripServo.setPosition(INTAKE_GRIP_STATE.OPEN.gripPosition);
        if (autoIntakeCloseMode) {
            intakeGripState = INTAKE_GRIP_STATE.OPEN_AUTO;
        } else {
            intakeGripState = INTAKE_GRIP_STATE.OPEN;
        }
    }
    /**
     * If state of hand grip is set to close, set position of servo's to specified
     */
    public void closeGrip(){
        intakeGripServo.setPosition(INTAKE_GRIP_STATE.CLOSED.gripPosition);
        intakeGripState = INTAKE_GRIP_STATE.CLOSED;
    }

    public double getIntakeGripColorSensorDistance(){
        return intakeGripDistance;
    }

    public boolean isIntakeArmInState(INTAKE_ARM_STATE toArmState) {
        return ((intakeArmState == toArmState) &&
                Math.abs(intakeArmServoLeft.getPosition() - toArmState.leftArmPosition) <= 0.03);
    }

    public boolean isIntakeGripInState(INTAKE_GRIP_STATE toGripState) {
        return ((intakeGripState == toGripState) &&
                Math.abs(intakeGripServo.getPosition() - toGripState.gripPosition) <= 0.05);
    }



}
