package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.GameOpModes.OldAuto.GameField;



public class OuttakeSlides {
    //Initialization of outtakemotor
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;


    //Outtake Motor : 5202 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 6mm D-Shaft, 435 RPM, ⌀36mm Gearbox, 3.3 - 5V Encoder)
    public static final double OUTTAKE_MOTOR_ENCODER_TICKS = 384.5;
    public static final double ADJUST_RATIO = 1;

    //Outtake Motor states
    public enum OUTTAKE_SLIDE_STATE {
        MIN_RETRACTED (0), //Position
        TRANSFER(0),
        PICKUP(0),
        READY_FOR_TRANSFER(0), //500
        //TRAVEL(0),
        DROP_LOWEST_AUTO(450),
        DROP_LOWEST(700), //570//88
        DROP_LOW_WHITE_AUTO(700),
        DROP_LOW_LINE(950), //830//190
        DROP_BELOW_MID(1168),//1168//292
        DROP_LEVEL_MID(1475),//1475//394
        DROP_BELOW_HIGH(1744),//1744//496
        DROP_LEVEL_HIGH(2077),//2077//598
        DROP_HIGHEST(2250),//2250//700
        MAX_EXTENDED(2280),//2280//700
        RANDOM(0);

        public final double motorPosition;
        OUTTAKE_SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public OUTTAKE_SLIDE_STATE outtakeSlidesState = OUTTAKE_SLIDE_STATE.TRANSFER;

    public int outtakeMotorCurrentPosition = 0;
    public int outtakeMototLeftCurrentPosition, outtakeMotorRightCurrentPosition;
    public double outtakeMotorNewPosition = outtakeSlidesState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 50;//100
    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 50;//200

    //Different constants of arm speed
    public static final double OUTTAKE_MOTOR_POWER_TELEOP = 1.0;//0.75
    public static final double OUTTAKE_MOTOR_POWER_AUTO = 1.0;//0.75
    public static final double OUTTAKE_MOTOR_POWER_TO_MAGAZINE = 1.0;//1
    public enum OUTTAKE_MOVEMENT_DIRECTION {
        EXTEND,
        RETRACT
    }
    public OUTTAKE_MOVEMENT_DIRECTION outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;

    public double deltaCount = 0; //Need tested value

    public boolean runOuttakeMotorToLevelState = false;

    public Telemetry telemetry;
    //Constructor`
    public OuttakeSlides(HardwareMap hardwareMap, Telemetry telemetry){
        this.telemetry = telemetry;
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_slides_left");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_slides_right");
        initOuttakeSlides();
    }

    //Method is able to initialize the arm
    public void initOuttakeSlides(){
        resetOuttakeMotorMode();
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorLeft.setPositionPIDFCoefficients(10.0);
        outtakeMotorRight.setPositionPIDFCoefficients(10.0); //5
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
        turnOuttakeBrakeModeOff();
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            outtakeSlidesState = OUTTAKE_SLIDE_STATE.TRANSFER;
        } else {
            outtakeSlidesState = OUTTAKE_SLIDE_STATE.READY_FOR_TRANSFER;
        }
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeMotorLeft.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotorRight.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }


    public int errorRightMinusLeftPosition = 0;
    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(OUTTAKE_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeMototLeftCurrentPosition = outtakeMotorLeft.getCurrentPosition();
        outtakeMotorRightCurrentPosition = outtakeMotorRight.getCurrentPosition();
        errorRightMinusLeftPosition = outtakeMotorRightCurrentPosition - outtakeMototLeftCurrentPosition;
        outtakeMotorLeft.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeMotorRight.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void modifyOuttakeSlidesLengthContinuous(double power){
        outtakeMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnOuttakeBrakeModeOn();
        double outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
        if ((power > 0.01 && outtakeMotorCurrentPosition < OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition) ||
                (power < -0.01 && outtakeMotorCurrentPosition > OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition )) {
            outtakeMotorLeft.setPower(power);
            outtakeMotorRight.setPower(power);
        } else {
            outtakeMotorLeft.setPower(0);
            outtakeMotorRight.setPower(0);
        }
    }

    public void modifyOuttakeSlidesLengthInSteps(double direction){
        deltaCount = direction * OUTTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
            outtakeMotorNewPosition = (outtakeMotorCurrentPosition + deltaCount);
            if (outtakeMotorNewPosition < OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition;
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
            } else if (outtakeMotorNewPosition > OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition) {
                outtakeMotorNewPosition = OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition;
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.MAX_EXTENDED;
            } else {
                outtakeSlidesState = OUTTAKE_SLIDE_STATE.RANDOM;
            }
            outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
            if (outtakeMotorCurrentPosition < outtakeMotorNewPosition ) {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (outtakeMotorNewPosition != outtakeMotorCurrentPosition) {
                turnOuttakeBrakeModeOn();
                outtakeMotorLeft.setTargetPosition((int)outtakeMotorNewPosition);
                outtakeMotorRight.setTargetPosition((int)outtakeMotorNewPosition);
                runOuttakeMotorToLevelState = true;
            }
        }
        if (runOuttakeMotorToLevelState) {
            runOuttakeMotorToLevel();
        }
    }

    //sets the Outtake motor power
    public void runOuttakeMotorToLevel(){
        double power = 0;
        if (outtakeSlidesState == OUTTAKE_SLIDE_STATE.MIN_RETRACTED) {
            turnOuttakeBrakeModeOff();
        } else {
            turnOuttakeBrakeModeOn();
        }
        if (GameField.opModeRunning == GameField.OP_MODE_RUNNING.HAZMAT_AUTONOMOUS) {
            power = OUTTAKE_MOTOR_POWER_AUTO;
        } else {
            power = OUTTAKE_MOTOR_POWER_TELEOP;
        }
        if (outtakeSlidesState == OUTTAKE_SLIDE_STATE.TRANSFER) {
            power = OUTTAKE_MOTOR_POWER_TO_MAGAZINE;
        }
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runOuttakeMotorToLevelState == true){
            outtakeMotorLeft.setPower(power);
            outtakeMotorRight.setPower(power);
            runOuttakeMotorToLevelState = false;
        } else{
            outtakeMotorLeft.setPower(0.0);
            outtakeMotorRight.setPower(0.0);
        }

        //TODO: Protect motor with checking of Voltage Sensor to determine stalling

    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode1 = outtakeMotorLeft.getMode();
        DcMotorEx.RunMode runMode2 = outtakeMotorRight.getMode();
        outtakeMotorLeft.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorRight.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotorLeft.setMode(runMode1);
        outtakeMotorRight.setMode(runMode2);
        outtakeMotorLeft.setPositionPIDFCoefficients(5.0);
        outtakeMotorRight.setPositionPIDFCoefficients(5.0); //5
        outtakeMotorLeft.setDirection(DcMotorEx.Direction.REVERSE);
        outtakeMotorRight.setDirection(DcMotorEx.Direction.FORWARD);
    }

    //TODO : Add logic to use Voltage Sensor to measure motor stalling and reset.
    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        //while (outtakeTouch.getState() && timer.time() < 5000) { TODO : Do it without touch sensor
        outtakeMotorLeft.setTargetPosition((int) (outtakeMotorLeft.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeMotorRight.setTargetPosition((int) (outtakeMotorRight.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        //}
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE toOuttakeSlideState) {
        //isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeMotorRight.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 15);
    }

    public boolean isOuttakeSlidesInStateDrop() {
        OUTTAKE_SLIDE_STATE toOuttakeSlideState = outtakeSlidesState;
        if (outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_LOW_LINE ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_LOWEST ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_LEVEL_MID ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_BELOW_MID ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_LEVEL_HIGH ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_BELOW_HIGH ||
                outtakeSlidesState == OUTTAKE_SLIDE_STATE.DROP_HIGHEST) {
            isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
            return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 30);
        } else {
            return false;
        }
    }



    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Outtake Slides");
        telemetry.addData("    State", outtakeSlidesState);
        telemetry.addData("    Left Motor Position", outtakeMotorLeft.getCurrentPosition());
        telemetry.addData("    Right Motor Position", outtakeMotorRight.getCurrentPosition());
        telemetry.addLine("=============");
    }

}