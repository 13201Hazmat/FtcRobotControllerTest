package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    //Initialization of intakemotor
    public DcMotorEx climberMotor = null;
    public CRServo climberLiftServo;

    public double CLIMBER_SERVO_POWER = 1.0;
    public boolean climberServoRunning = false;

    //Outtake Motor states
    public enum CLIMBER_MOTOR_STATE {
        INITIAL_STATE(0), //Position
        CLIMBED_STATE(-3000); //117 rpm motor TODO Set value

        public final double motorPosition;
        CLIMBER_MOTOR_STATE(double motorPosition) {
            this.motorPosition = motorPosition;
        }

    }
    public CLIMBER_MOTOR_STATE climberMotorState = CLIMBER_MOTOR_STATE.INITIAL_STATE;

    public double climberMotorCurrentPosition = climberMotorState.motorPosition;
    public double climberMotorNewPosition = climberMotorState.motorPosition;

    //Different constants of arm speed
    public static final double CLIMBER_MOTOR_POWER_TELEOP = 1;

    public boolean runClimberMotorToLevelState = false;

    public double climberMotorPower = CLIMBER_MOTOR_POWER_TELEOP;
    public double startCurrentPosition;

    public Telemetry telemetry;
    public Climber(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        climberMotor = hardwareMap.get(DcMotorEx.class, "leftBack");//climber_motor
        climberLiftServo = hardwareMap.get(CRServo.class, "climber_lift");

        initClimber();
    }

    public void initClimber(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberMotor.setPositionPIDFCoefficients(10.0); //5
        climberMotor.setDirection(DcMotorEx.Direction.REVERSE);
        startCurrentPosition = climberMotor.getCurrentPosition();
        turnClimberBrakeModeOff();
    }

    public void moveClimberSlidesUp(){
        climberLiftServo.setPower(CLIMBER_SERVO_POWER);
        climberServoRunning = true;
    }

    public void moveClimberSlidesDown(){
        climberLiftServo.setPower(-CLIMBER_SERVO_POWER);
        climberServoRunning = true;
    }

    public void stopClimberSlides(){
        climberLiftServo.setPower(0);
        climberServoRunning = false;
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
        climberMotor.setTargetPosition((int)(toClimberMotorState.motorPosition + startCurrentPosition));
        climberMotorState = toClimberMotorState;
        runClimberMotorToLevelState = true;
        runClimberMotorToLevel();
    }

    //sets the Outtake motor power
    public void runClimberMotorToLevel(){
        double power = 0;
        if (climberMotorState == CLIMBER_MOTOR_STATE.INITIAL_STATE) {
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
    }

    public void modifyClimberMotorLength(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberBrakeModeOn();

        double climberMotorCurrentPosition = climberMotor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBER_MOTOR_STATE.CLIMBED_STATE.motorPosition) ||
                (power < -0.01 && climberMotorCurrentPosition > CLIMBER_MOTOR_STATE.INITIAL_STATE.motorPosition)){
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

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addData("Climber Motor State", climberMotorState);
        telemetry.addData("Climber Motor Position", climberMotor.getCurrentPosition());
        telemetry.addLine("=============");
    }

}
