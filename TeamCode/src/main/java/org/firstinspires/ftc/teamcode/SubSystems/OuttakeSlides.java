package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

        import com.qualcomm.robotcore.hardware.DcMotorEx;
        import com.qualcomm.robotcore.hardware.DcMotorSimple;
        import com.qualcomm.robotcore.hardware.HardwareMap;
        import com.qualcomm.robotcore.hardware.Servo;
        import com.qualcomm.robotcore.util.ElapsedTime;

        import org.firstinspires.ftc.robotcore.external.Telemetry;
        import org.firstinspires.ftc.teamcode.GameOpModes.GameField;



public class OuttakeSlides {
    //Initialization of outtakemotor
    public DcMotorEx outtakeMotor1;
    public DcMotorEx outtakeMotor2;


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
        outtakeMotor1 = hardwareMap.get(DcMotorEx.class, "outtake_motor1");
        outtakeMotor2 = hardwareMap.get(DcMotorEx.class, "outtake_motor2");
        initOuttakeSlides();
    }

    //Method is able to initialize the arm
    public void initOuttakeSlides(){
        outtakeMotor2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeMotor2.setPositionPIDFCoefficients(10.0); //5
        outtakeMotor2.setDirection(DcMotorEx.Direction.REVERSE);
        turnOuttakeBrakeModeOff();
        //manualResetOuttakeMotor();
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOn(){
        outtakeMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        outtakeMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnOuttakeBrakeModeOff(){
        outtakeMotor1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        outtakeMotor2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveOuttakeSlides(OUTTAKE_SLIDE_STATE toOuttakeMotorState){
        turnOuttakeBrakeModeOn();
        outtakeMotorCurrentPosition = outtakeMotor1.getCurrentPosition();
        outtakeMotorCurrentPosition = outtakeMotor2.getCurrentPosition();
        /*if (outtakeMotorCurrentPosition < toOuttakeMotorState.motorPosition ) {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
        } else {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
        }
         */
        outtakeMotor1.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeMotor2.setTargetPosition((int)toOuttakeMotorState.motorPosition);
        outtakeSlidesState = toOuttakeMotorState;
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
    }

    public void modifyOuttakeSlidesLength(double stepSizeFactor){
        deltaCount = stepSizeFactor * OUTTAKE_MOTOR_DELTA_COUNT_MAX;
        if (deltaCount !=0) {
            outtakeMotorCurrentPosition = outtakeMotor1.getCurrentPosition();
            outtakeMotorCurrentPosition = outtakeMotor2.getCurrentPosition();
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
            outtakeMotorCurrentPosition = outtakeMotor1.getCurrentPosition();
            outtakeMotorCurrentPosition = outtakeMotor2.getCurrentPosition();
            if (outtakeMotorCurrentPosition < outtakeMotorNewPosition ) {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
            } else {
                outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
            }
            if (outtakeMotorNewPosition != outtakeMotorCurrentPosition) {
                turnOuttakeBrakeModeOn();
                outtakeMotor1.setTargetPosition((int)outtakeMotorNewPosition);
                outtakeMotor2.setTargetPosition((int)outtakeMotorNewPosition);
                runOuttakeMotorToLevelState = true;
            }
        }
    }

    /*
    public void modifyOuttakeSlidesLength(double power){
        //outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnOuttakeBrakeModeOn();
        double outtaMotorCurrentPosition = outtakeMotor.getCurrentPosition();
        if ((power > 0.01 && outtaMotorCurrentPosition < OUTTAKE_SLIDE_STATE.MAX_EXTENDED.motorPosition) ||
                (power < 0.01 && outtaMotorCurrentPosition > OUTTAKE_SLIDE_STATE.MIN_RETRACTED.motorPosition )) {
            outtakeMotor.setPower(power);
        } else {
            outtakeMotor.setPower(0);
        }
    }
     */


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
        outtakeMotor1.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        outtakeMotor2.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        if (runOuttakeMotorToLevelState == true){
            outtakeMotor1.setPower(power);
            outtakeMotor2.setPower(power);
            /*TODO if (!outtakeMotor.isBusy()) */runOuttakeMotorToLevelState = false;
        } else{
            outtakeMotor1.setPower(0.0);
            outtakeMotor2.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO
        if (!outtakeTouch.getState()) {
            resetOuttakeMotorMode();
        }
         */
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode1 = outtakeMotor1.getMode();
        DcMotorEx.RunMode runMode2 = outtakeMotor2.getMode();
        outtakeMotor1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor1.setMode(runMode1);
        outtakeMotor2.setMode(runMode2);
    }

    public void manualResetOuttakeMotor(){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        //while (outtakeTouch.getState() && timer.time() < 5000) { TODO : Do it without touch sensor
        outtakeMotor1.setTargetPosition((int) (outtakeMotor1.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        outtakeMotor2.setTargetPosition((int) (outtakeMotor2.getCurrentPosition() - OUTTAKE_MOTOR_DELTA_COUNT_RESET));
        runOuttakeMotorToLevelState = true;
        runOuttakeMotorToLevel();
        //}
        resetOuttakeMotorMode();
        turnOuttakeBrakeModeOff();
        outtakeSlidesState = OUTTAKE_SLIDE_STATE.MIN_RETRACTED;
    }

    public double isOuttakeSlidesInStateError = 0;
    public boolean isOuttakeSlidesInState(OUTTAKE_SLIDE_STATE toOuttakeSlideState) {
        isOuttakeSlidesInStateError = Math.abs(outtakeMotor1.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        isOuttakeSlidesInStateError = Math.abs(outtakeMotor2.getCurrentPosition() - toOuttakeSlideState.motorPosition);
        return (outtakeSlidesState == toOuttakeSlideState && isOuttakeSlidesInStateError <= 30);
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("=============");
    }

}