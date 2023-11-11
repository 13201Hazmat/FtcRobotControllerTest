package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

        import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.GameOpModes.GameField;



public class OuttakeSlides {
    //Initialization of outtakemotor
    public DcMotorEx outtakeMotorLeft;
    public DcMotorEx outtakeMotorRight;


    //Outtake Motor : 5202 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 24mm Length 6mm D-Shaft, 435 RPM, ⌀36mm Gearbox, 3.3 - 5V Encoder)
    public static final double OUTTAKE_MOTOR_ENCODER_TICKS = 145.1;//384.5;
    public static final double ADJUST_RATIO = 145.1/384.5;

    //public DigitalChannel outtakeTouch;  // Hardware Device Object


    //Outtake Motor states
    public enum OUTTAKE_SLIDE_STATE {
        MIN_RETRACTED (0), //Position
        LEVEL_LOW(500),
        LEVEL_MID(1000),
        LEVEL_HIGH(1500),
        MAX_EXTENDED(1600), //1600 //975 for 1150 rpm
        RANDOM(0);

        public final double motorPosition;
        OUTTAKE_SLIDE_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public OUTTAKE_SLIDE_STATE outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;

    public double outtakeMotorCurrentPosition = outtakeSlidesState.motorPosition;
    public double outtakeMotorNewPosition = outtakeSlidesState.motorPosition;

    public static final double OUTTAKE_MOTOR_DELTA_COUNT_MAX = 100;//100
    public static final double OUTTAKE_MOTOR_DELTA_COUNT_RESET = 200;//200

    //Different constants of arm speed
    public static final double OUTTAKE_MOTOR_POWER_TELEOP = 1;
    public static final double OUTTAKE_MOTOR_POWER_AUTO = 1;
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
        outtakeMotorLeft = hardwareMap.get(DcMotorEx.class, "outtake_motor1");
        outtakeMotorRight = hardwareMap.get(DcMotorEx.class, "outtake_motor2");
        initOuttakeSlides();
    }

    //Method is able to initialize the arm
    public void initOuttakeSlides(){
        outtakeMotorRight.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotorRight.setPositionPIDFCoefficients(10.0); //5
        outtakeMotorRight.setDirection(DcMotorEx.Direction.REVERSE);
        turnOuttakeBrakeModeOff();
        //manualResetOuttakeMotor();
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

    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(OUTTAKE_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
        outtakeMotorCurrentPosition = outtakeMotorRight.getCurrentPosition();
        /*if (outtakeMotorCurrentPosition < toOuttakeMotorState.motorPosition ) {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
        } else {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
        }
         */
        outtakeMotorLeft.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeMotorRight.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void modifyOuttakeSlidesLength(double direction){
        deltaCount = direction * OUTTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            outtakeMotorCurrentPosition = outtakeMotorLeft.getCurrentPosition();
            outtakeMotorCurrentPosition = outtakeMotorRight.getCurrentPosition();
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
            outtakeMotorCurrentPosition = outtakeMotorRight.getCurrentPosition();
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
        isOuttakeSlidesInStateError = Math.abs(outtakeMotorLeft.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeMotorRight.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 30);
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addData("Outtake Slides State", outtakeSlidesState);
        telemetry.addData("Outtake Motor Left Position", outtakeMotorLeft.getCurrentPosition());
        telemetry.addData("Outtake Motor Right Position", outtakeMotorRight.getCurrentPosition());
        telemetry.addLine("=============");
    }

}