package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class IntakeSlides {

    //Initialization of intakemotor
    public DcMotorEx intakeMotorLeft, intakeMotorRight;

    //Intake Motor : 5203 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 8mm REX Shaft, 435 RPM, 3.3 - 5V Encoder)
    public static final double INTAKE_MOTOR_ENCODER_TICKS = 145.6;//384.5;

    //public DigitalChannel intakeTouch;  // Hardware Device Object
    public TouchSensor intakeTouch;

    public DistanceSensor intakeDistanceSensor;

    //Intake Motor states
    public enum INTAKE_SLIDES_STATE {
        MIN_RETRACTED (0,0), //Position
        TRANSFER (0, 0),
        MAX_EXTENDED(666, 6), //1760
        RANDOM(0, 7),
        
        AUTO_CONE_1(480, 1), //490
        AUTO_CONE_2(445, 2), //455
        AUTO_CONE_3(435, 3), //445
        AUTO_COME_4(423, 4), //433
        AUTO_CONE_5(413, 5); //423

        public final double motorPosition;
        public final int index;
        INTAKE_SLIDES_STATE(double motorPosition, int index) {

            this.motorPosition = motorPosition;
            this.index = index;
        }

        public INTAKE_SLIDES_STATE byIndex(int ord) {
            if (ord <1) ord = 1;
            if (ord >5) ord = 5;
            for (INTAKE_SLIDES_STATE a : INTAKE_SLIDES_STATE.values()) {
                if (a.index == ord) {
                    return a;
                }
            }
            return null;
        }

    }
    public INTAKE_SLIDES_STATE intakeSlidesState = INTAKE_SLIDES_STATE.MIN_RETRACTED;

    public double intakeMotorCurrentPosition = intakeSlidesState.motorPosition;
    public double intakeMotorNewPosition = intakeSlidesState.motorPosition;

       //Different constants of arm speed
    public static double INTAKE_MOTOR_DELTA_COUNT_RESET = 200;
    public static final double INTAKE_MOTOR_POWER_TELEOP = 1.0;
    public static final double INTAKE_MOTOR_POWER_RESET = 0.8;
    public static final double INTAKE_MOTOR_POWER_AUTO = 0.75;
    public enum INTAKE_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }
    public IntakeSlides.INTAKE_MOVEMENT_DIRECTION intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.RETRACT;

    public boolean runIntakeMotorToLevelState = false;

    //Constructor`
    public IntakeSlides(HardwareMap hardwareMap){
        intakeMotorLeft = hardwareMap.get(DcMotorEx.class, "intake_motor_left");
        intakeMotorRight = hardwareMap.get(DcMotorEx.class, "intake_motor_right");

        intakeTouch = hardwareMap.get(TouchSensor.class,"intake_reset_ts");
        initIntakeSlides();
    }

    //Method is able to initialize the arm
    public void initIntakeSlides(){
        resetIntakeMotorMode();
        intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorLeft.setPositionPIDFCoefficients(5.0);
        intakeMotorRight.setPositionPIDFCoefficients(5.0);
        intakeMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        turnIntakeBrakeModeOn();
        moveIntakeSlidesToMinRetracted(); //TODO : CAUSING BATTERY DISCHARGE
    }

    //Turns on the brake for Intake motor
    public void turnIntakeBrakeModeOn(){
        intakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        intakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Intake motor
    public void turnIntakeBrakeModeOff(){
        intakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        intakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets intake slides to position
    public void moveIntakeSlides(INTAKE_SLIDES_STATE toIntakeMotorState){
        if (toIntakeMotorState == INTAKE_SLIDES_STATE.TRANSFER) {
            moveIntakeSlidesToMinRetracted();
            intakeSlidesState = INTAKE_SLIDES_STATE.TRANSFER;
        } else {
            turnIntakeBrakeModeOn();
            intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
            if (intakeMotorCurrentPosition < toIntakeMotorState.motorPosition) {
                intakeMovementDirection = INTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                intakeMovementDirection = INTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            intakeMotorLeft.setTargetPosition((int) toIntakeMotorState.motorPosition);
            intakeMotorRight.setTargetPosition((int) toIntakeMotorState.motorPosition);
            intakeSlidesState = toIntakeMotorState;
            runIntakeMotorToLevelState = true;
        }
    }

    /*public void modifyIntakeSlidesLength1(double stepSizeFactor, int direction){
        deltaCount = stepSizeFactor * INTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
            intakeMotorNewPosition = (intakeMotorCurrentPosition + direction * deltaCount);
            if (intakeMotorNewPosition < IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition) {
                intakeMotorNewPosition = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED.motorPosition;
                intakeSlidesState = IntakeSlides.INTAKE_MOTOR_STATE.MIN_RETRACTED;
            } else if (intakeMotorNewPosition > IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition) {
                intakeMotorNewPosition = IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED.motorPosition;
                intakeSlidesState = IntakeSlides.INTAKE_MOTOR_STATE.MAX_EXTENDED;
            } else {
                intakeSlidesState = IntakeSlides.INTAKE_MOTOR_STATE.RANDOM;
            }
            intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
            if (intakeMotorCurrentPosition < intakeMotorNewPosition ) {
                intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                intakeMovementDirection = IntakeSlides.INTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (intakeMotorNewPosition != intakeMotorCurrentPosition) {
                turnIntakeBrakeModeOn();
                intakeMotorLeft.setTargetPosition((int)intakeMotorNewPosition);
                intakeMotorRight.setTargetPosition((int)intakeMotorNewPosition);
                runIntakeMotorToLevelState = true;
            }
        }
    }
*/
    public void modifyIntakeSlidesLength(double power){
        intakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnIntakeBrakeModeOn();
        double intakeMotorCurrentPosition = intakeMotorLeft.getCurrentPosition();
        if ((power > 0.01 && intakeMotorCurrentPosition < INTAKE_SLIDES_STATE.MAX_EXTENDED.motorPosition) ||
            (power < -0.01 && intakeMotorCurrentPosition > INTAKE_SLIDES_STATE.MIN_RETRACTED.motorPosition )) {
            intakeMotorLeft.setPower(power);
            intakeMotorRight.setPower(power);
        } else {
            intakeMotorLeft.setPower(0);
            intakeMotorRight.setPower(0);
        }
    }


    //sets the Intake motor power
    public void runIntakeMotorToLevel(){
        double power = 0;
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS
                &&  intakeMovementDirection == INTAKE_MOVEMENT_DIRECTION.EXTEND) {
            power = INTAKE_MOTOR_POWER_AUTO;
        } else {
            power = INTAKE_MOTOR_POWER_TELEOP;
        }
        turnIntakeBrakeModeOn();
        intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runIntakeMotorToLevelState){
            intakeMotorLeft.setPower(power);
            intakeMotorRight.setPower(power);
            /*TODO if(!intakeMotorLeft.isBusy())*/ runIntakeMotorToLevelState = false;
        } else {
            intakeMotorLeft.setPower(0.0);
            intakeMotorRight.setPower(0.0);
        }

        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO */
        /*if (!intakeTouch.getState()) {
            resetIntakeMotorMode();
        }*/
    }

    //Resets the arm
    public void resetIntakeMotorMode(){
        intakeMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intakeMotorLeft.setPositionPIDFCoefficients(5.0);
        intakeMotorRight.setPositionPIDFCoefficients(5.0);
        intakeMotorLeft.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        turnIntakeBrakeModeOn();

    }

    public void moveIntakeSlidesToMinRetracted(){
        ElapsedTime timer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        timer.reset();
        //turnIntakeBrakeModeOff();
        while (!intakeTouch.isPressed() && timer.time() < 1000) {
            intakeMotorLeft.setTargetPosition((int) (intakeMotorLeft.getCurrentPosition() - INTAKE_MOTOR_DELTA_COUNT_RESET));
            intakeMotorRight.setTargetPosition((int) (intakeMotorRight.getCurrentPosition() - INTAKE_MOTOR_DELTA_COUNT_RESET));
            //runIntakeMotorToLevelState = true;
            intakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
            intakeMotorLeft.setPower(INTAKE_MOTOR_POWER_RESET);
            intakeMotorRight.setPower(INTAKE_MOTOR_POWER_RESET);
        }
        resetIntakeMotorMode();
        turnIntakeBrakeModeOn();
        intakeMotorLeft.setPower(0.0);
        intakeMotorRight.setPower(0.0);
        intakeSlidesState = INTAKE_SLIDES_STATE.MIN_RETRACTED;
    }



    public double getDistance(){
        return intakeDistanceSensor.getDistance(DistanceUnit.MM);
    }

    //TODO Protect intake so that it does not hit anything based on distance read by Distance Sensor


    public boolean isIntakeSlidesInState(INTAKE_SLIDES_STATE toIntakeSlidesState) {
        if (toIntakeSlidesState == INTAKE_SLIDES_STATE.TRANSFER ||
            toIntakeSlidesState == INTAKE_SLIDES_STATE.MIN_RETRACTED) {
            return intakeTouch.isPressed();
        } else {
            return ( (intakeSlidesState == toIntakeSlidesState) &&
                    Math.abs(intakeMotorLeft.getCurrentPosition() - toIntakeSlidesState.motorPosition)
                            <= 30);
        }
    }
}

