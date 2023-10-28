package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

public class Climber {
    //Initialization of intakemotor
    public DcMotorEx climberMotor = null;
    public Servo climberLiftLeft;
    public Servo climberLiftRight;

    public enum CLIMBER_SLIDES_HEIGHT{
        CLIMBER_SLIDES_LIFTED(0,0), //UPDATE FOR THIS YEAR
        CLIMBER_SLIDES_DROPPED(0,0);

        private double liftLeftPosition;
        private double liftRightPosition;


        CLIMBER_SLIDES_HEIGHT(double moveLeftPosition, double moveRightPosition){
               this.liftLeftPosition = moveLeftPosition;
                this.liftRightPosition = moveRightPosition;
        }
        public double getLiftLeftPosition(){
            return liftLeftPosition;
        }
        public double getLiftRightPosition(){
            return liftRightPosition;
        }
    }
    public CLIMBER_SLIDES_HEIGHT climberSlidesHeightState = CLIMBER_SLIDES_HEIGHT.CLIMBER_SLIDES_DROPPED;

    //Outtake Motor states
    public enum CLIMBER_MOTOR_STATE {
        MIN_RETRACTED (0), //Position
        MAX_EXTENDED(1600), //1600 //975 for 1150 rpm
        RANDOM(0);

        public final double motorPosition;
        CLIMBER_MOTOR_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.MIN_RETRACTED;

    public double climberMotorCurrentPosition = climberMotorState.motorPosition;
    public double climberMotorNewPosition = climberMotorState.motorPosition;

    //Different constants of arm speed
    public static final double CLIMBER_MOTOR_POWER_TELEOP = 1;

    public boolean runClimberMotorToLevelState = false;

    public double climberMotorPower = CLIMBER_MOTOR_POWER_TELEOP;

    public Climber(HardwareMap hardwareMap) {
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");
        climberLiftLeft = hardwareMap.get(Servo.class, "climber_lift_left");
        climberLiftRight = hardwareMap.get(Servo.class, "climber_lift_right");
        initIntake();
    }

    public void initIntake(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberMotor.setPositionPIDFCoefficients(10.0); //5
        climberMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turnClimberBrakeModeOff();
        climberSlidesHeightState = CLIMBER_SLIDES_HEIGHT.CLIMBER_SLIDES_DROPPED;
        moveRollerHeight(CLIMBER_SLIDES_HEIGHT.CLIMBER_SLIDES_DROPPED);
    }

    public void moveRollerHeight(CLIMBER_SLIDES_HEIGHT climberSlidesHeight){
        climberLiftLeft.setPosition(climberSlidesHeight.liftLeftPosition);
        climberLiftRight.setPosition(climberSlidesHeight.liftRightPosition);
        climberSlidesHeightState = climberSlidesHeight;
    }

    public void initClimberSlides(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberMotor.setPositionPIDFCoefficients(10.0); //5
        climberMotor.setDirection(DcMotorEx.Direction.REVERSE);
        turnClimberBrakeModeOff();
        //manualResetOuttakeMotor();
    }

    //Turns on the brake for Outtake motor
    public void turnClimberBrakeModeOn(){
        climberMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    //Turns on the brake for Outtake motor
    public void turnClimberBrakeModeOff(){
        climberMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    //Sets outtake slides to Transfer position
    public void moveClimberMotor(CLIMBER_MOTOR_STATE toClimberMotorState){
        turnClimberBrakeModeOn();
        climberMotorCurrentPosition = climberMotor.getCurrentPosition();
        /*if (outtakeMotorCurrentPosition < toOuttakeMotorState.motorPosition ) {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.EXTEND;
        } else {
            outtakeMovementDirection = OUTTAKE_MOVEMENT_DIRECTION.RETRACT;
        }
         */
        climberMotor.setTargetPosition((int)toClimberMotorState.motorPosition);
        climberMotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberMotorToLevel();
    }

    //sets the Outtake motor power
    public void runClimberMotorToLevel(){
        double power = 0;
        if (climberMotorState == CLIMBER_MOTOR_STATE.MIN_RETRACTED) {
            turnClimberBrakeModeOff();
        } else {
            turnClimberBrakeModeOn();
        }
        power = CLIMBER_MOTOR_POWER_TELEOP;

        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runClimberMotorToLevelState == true){
            climberMotor.setPower(power);
            /*TODO if (!outtakeMotor.isBusy()) */runClimberMotorToLevelState = false;
        } else{
            climberMotor.setPower(0.0);
        }
        /*if ((armState == ARM_STATE.MIN_RETRACTED) && (armTouchSensor.getState())){
            manualResetArm();
        } Overheating arm? TODO
        if (!outtakeTouch.getState()) {
            resetOuttakeMotorMode();
        }
         */
    }

    public void modifyClimberSlidesLength(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberBrakeModeOn();

        double climberMotorCurrentPosition = climberMotor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBER_MOTOR_STATE.MAX_EXTENDED.motorPosition) ||
                (power < -0.01 && climberMotorCurrentPosition > CLIMBER_MOTOR_STATE.MIN_RETRACTED.motorPosition)){
            climberMotor.setPower(power);
        } else {
            climberMotor.setPower(0);
        }
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode = climberMotor.getMode();
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberMotor.setMode(runMode);
    }

    public double isClimberMotorInStateError = 0;
    public boolean isClimberMotorInState(CLIMBER_MOTOR_STATE toClimberMotorState) {
        isClimberMotorInStateError = Math.abs(climberMotor.getCurrentPosition() - toClimberMotorState.motorPosition);
        isClimberMotorInStateError = Math.abs(climberMotor.getCurrentPosition() - toClimberMotorState.motorPosition);
        return (climberMotorState == toClimberMotorState && isClimberMotorInStateError <= 30);
    }


}
