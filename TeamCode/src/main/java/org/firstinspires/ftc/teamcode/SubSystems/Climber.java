package org.firstinspires.ftc.teamcode.SubSystems;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Climber {
    //Initialization of intakemotor
    public DcMotorEx climberMotor = null;
    public CRServo climberLiftServo;
    //public Servo climberServo;

    public double CLIMBER_SERVO_POWER = 1.0;
    public boolean climberServoRunning = false;
    public boolean climberActivate = false;

    //Outtake Motor states
    public enum CLIMBER_MOTOR_STATE {
        INITIAL(0), //Position
        CLIMBED(-3000); //117 rpm motor TODO Set value

        public final double motorPosition;
        CLIMBER_MOTOR_STATE(double motorPosition) {
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
        //climberServo = hardwareMap.get(Servo.class, "climber_lift_servo");

        initClimber();
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

    /*public enum CLIMBER_SERVO_STATE {
        CLIMBER_HELD(0.26), //TODO : Update Value
        CLIMBER_RELEASED(0.5); //TODO : Update Value

        private double climberServoPosition;

       CLIMBER_SERVO_STATE(double moveClimberPosition){
            this.climberServoPosition = moveClimberPosition;
        }

        public double getClimberPosition(){return climberServoPosition;}
    }
    public CLIMBER_SERVO_STATE climberServoState = CLIMBER_SERVO_STATE.CLIMBER_HELD;
    */

    public enum CLIMBER_SERVO_STATE{
        LOW,
        HIGH
    }
    public CLIMBER_SERVO_STATE ClimberServoState = CLIMBER_SERVO_STATE.LOW;

    /*public enum CLIMBER_BUTTON_STATE{
        SAFE,
        ARMED,
        RELEASED
    }
    public CLIMBER_BUTTON_STATE climberButtonState = CLIMBER_BUTTON_STATE.SAFE;
    public ElapsedTime climberClickTimer = new ElapsedTime(MILLISECONDS);
    public double CLIMBER_BUTTON_ARMED_THRESHOLD = 300;

     */

    public void initClimber(){
        climberMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        climberMotor.setPositionPIDFCoefficients(10.0); //5
        climberMotor.setDirection(DcMotorEx.Direction.REVERSE);
        startCurrentPosition = climberMotor.getCurrentPosition();
        turnClimberBrakeModeOff();
        //climberServo.setPosition(CLIMBER_SERVO_STATE.CLIMBER_HELD.getClimberPosition());
        //climberServoState = CLIMBER_SERVO_STATE.CLIMBER_HELD;
        //climberButtonState = CLIMBER_BUTTON_STATE.SAFE;

    }

    /*public void releaseClimber(){
        climberServo.setPosition(CLIMBER_SERVO_STATE.CLIMBER_RELEASED.getClimberPosition());
        climberServoState = CLIMBER_SERVO_STATE.CLIMBER_RELEASED;
    }*/

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

    public void modifyClimberMotorLength(double power){
        climberMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        turnClimberBrakeModeOn();

        double climberMotorCurrentPosition = climberMotor.getCurrentPosition();
        if((power > 0.01 && climberMotorCurrentPosition < CLIMBER_MOTOR_STATE.CLIMBED.motorPosition) ||
                (power < -0.01 && climberMotorCurrentPosition > CLIMBER_MOTOR_STATE.INITIAL.motorPosition)){
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

    public void printDebugMessages(){
        //******  debug ******
        //telemetry.addData("xx", xx);
        telemetry.addLine("Climber");
        telemetry.addData("    State", climberMotorState);
        telemetry.addData("    Motor Position", climberMotor.getCurrentPosition());
        telemetry.addLine("=============");
    }

}
