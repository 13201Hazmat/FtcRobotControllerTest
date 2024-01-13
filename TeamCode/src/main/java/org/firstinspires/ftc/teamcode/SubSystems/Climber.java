package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    //Initialization of intakemotor
    public DcMotorEx climberMotor = null;
    public CRServo climberLiftServo;
    //public Servo climberServo;


    public double CLIMBER_SERVO_POWER = -1.0;
    public double CLIMBER_SERVO_HOLD_POWER = -0.3;
    public boolean climberServoRunning = false;
    public boolean climberActivated = false;
    public boolean climbingStarted = false;

    //Outtake Motor states
    public enum CLIMBER_MOTOR_STATE {
        INITIAL(0), //Position
        CLIMBED(4000), //5000, 117 rpm motor TODO Set value, 6000
        MAX(7500); //9000

        public final int motorPosition;
        CLIMBER_MOTOR_STATE(int motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.INITIAL;

    public double climberMotorCurrentPosition = climberMotorState.motorPosition;
    public double climberMotorNewPosition = climberMotorState.motorPosition;

    public boolean runClimberMotorToLevelState = false;

    public double startCurrentPosition;

    public Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");
        climberLiftServo = hardwareMap.get(CRServo.class, "climber_lift_servo");

        initClimber();
    }

    public void moveClimberSlidesUp(){
        climberLiftServo.setPower(CLIMBER_SERVO_POWER);
        climberServoRunning = true;
    }

    public void holdClimberSlidesUp(){
        climberLiftServo.setPower(CLIMBER_SERVO_HOLD_POWER);
        climberServoRunning = true;
    }

    public void stopClimberSlides(){
        climberLiftServo.setPower(0);
        climberServoRunning = false;
    }

    public enum CLIMBER_SERVO_STATE{
        LOW,
        HIGH
    }
    public CLIMBER_SERVO_STATE ClimberServoState = CLIMBER_SERVO_STATE.LOW;

    public void initClimber(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberMotor.setPositionPIDFCoefficients(10.0); //5
        startCurrentPosition = climberMotor.getCurrentPosition();
        turnClimberBrakeModeOff();
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
        climberMotor.setTargetPosition((int)(toClimberMotorState.motorPosition + startCurrentPosition));
        climberMotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberMotorToLevel();
    }

    //sets the Outtake motor power
    public void runClimberMotorToLevel(){
        double power = 0;
        if (climberMotorState == CLIMBER_MOTOR_STATE.INITIAL) {
            turnClimberBrakeModeOff();
        } else {
            turnClimberBrakeModeOn();
        }
        power = 1.0;

        climberMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        if (runClimberMotorToLevelState == true){
            climberMotor.setPower(power);
            /*TODO if (!outtakeMotor.isBusy()) */runClimberMotorToLevelState = false;
        } else{
            climberMotor.setPower(0.0);
        }
    }

    public void moveClimberUpInSteps(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberBrakeModeOn();

        int climberMotorCurrentPosition = climberMotor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBER_MOTOR_STATE.MAX.motorPosition)){
            climberMotor.setTargetPosition(climberMotorCurrentPosition + CLIMBER_MOTOR_STATE.CLIMBED.motorPosition);
            climberMotor.setPositionPIDFCoefficients(10.0);
            climberMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            climberMotor.setPower(power);
        } else {
            climberMotor.setPower(0);
        }
    }

    public void modifyClimberLengthContinuous(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberBrakeModeOn();
        climberMotor.setPower(power);
    }

    //Resets the arm
    public void resetOuttakeMotorMode(){
        DcMotorEx.RunMode runMode = climberMotor.getMode();
        climberMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        climberMotor.setMode(runMode);
    }

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Climber");
        telemetry.addData("    State", climberMotorState);
        telemetry.addData("    Motor Position", climberMotor.getCurrentPosition());
        telemetry.addLine("=============");
    }

}
