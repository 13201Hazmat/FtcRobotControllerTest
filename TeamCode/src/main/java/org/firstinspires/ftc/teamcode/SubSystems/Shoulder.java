package org.firstinspires.ftc.teamcode.SubSystems;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
/**
 * Definition of Subsystem Class <BR>
 *
 * Example : Intake consists of system provided intake controls and adds functionality to the selection made on intake. <BR>
 *
 * The states are as followed: <BR>
 *     INTAKE_SERVO_LEVEL1 for one state - example if intake motor is running, stopped, or reversing  <BR>
 *     INTAKE_SERVO_LEVEL2 for another state  = example if the intake is on or off  <BR>
 *
 * The functions are as followed: Example assumes a motor like an intake <BR>
 *     runIntakeMotor checks if the motor is not running and runs the intake  <BR>
 *     stopIntakeMotor checks if the intake has stopped and if its not, it sets the intake power to 0
 *     and sets intakeMotorState to INTAKE_SERVO_LEVEL1.STOPPED  <BR>
 *      startReverseIntakeMotor checks if the motor is not reversing, and sets the  motor to FORWARD, then also
 *     sets intake motor state to REVERSING <BR>
 */

public class Shoulder {
    //Initialization of <Fill>
    public DcMotorEx rshmotor, lshmotor;

    //Initialization of <Fill>
    public enum SHOULDER_MOTOR_POSITION {
        GROUND_JUNCTION,
        PICKUP,
        LOW_JUNCTION,
        MEDIUM_JUNCTION,
        HIGH_JUNCTION,
        PARKED
    }

    public enum SHOULDER_STATE {
        MAX_POSITION,
        MIN_POSITION
    }



    //Initialization of <Fill>
    public SHOULDER_MOTOR_POSITION shoulderMotorPosition;
    public SHOULDER_STATE shoulderState;


    public boolean runShoulderToLevelState = false;
    public int shoulderDeltaCount = 50; //Need tested value
    public SHOULDER_MOTOR_POSITION currentShoulderPosition = SHOULDER_MOTOR_POSITION.PARKED;
    public SHOULDER_MOTOR_POSITION previousShoulderPosition = SHOULDER_MOTOR_POSITION.PARKED;
    public double maxPosition = 1000; //Need tested values
    public double minPosition = -100; //Need tested values
    public int shoulderCurrentShoulderPositionCount = GROUND_JUNCTION; //Default shoulder position count

    public static int baselineEncoderCount = 0; //need tested values
    public static final int GROUND_JUNCTION = 200; //Need tested values
    public static final int PARKED = 0; //need tested values
    public static final int LOW_JUNCTION = 400; //need tested values
    public static final int MEDIUM_JUNCTION = 600; //need tested values
    public static final int HIGH_JUNCTION = 800; //need tested values
    public static final int MAX_DELTA = 200; //need tested values

    //Different constants of shoulder speed
    public double HIGH_POWER = 1.0;
    public double MED_POWER = 0.5;
    public double LOW_POWER = 0.2;

    //Constructor
    public Shoulder(HardwareMap hardwareMap){
        lshmotor = hardwareMap.get(DcMotorEx.class, "lshmotor");
        rshmotor = hardwareMap.get(DcMotorEx.class, "rshmotor");
        initShoulder();
    }

    //Method is able to <Fill>
    public void initShoulder(){
        resetShoulder();
        turnShoulderBrakeModeOff();
        lshmotor.setPositionPIDFCoefficients(5.0);
        rshmotor.setPositionPIDFCoefficients(5.0);
        lshmotor.setTargetPosition(PARKED);
        rshmotor.setTargetPosition(PARKED);
        lshmotor.setDirection(DcMotor.Direction.FORWARD);
        rshmotor.setDirection(DcMotor.Direction.REVERSE);
        currentShoulderPosition = shoulderMotorPosition.PARKED;
        previousShoulderPosition = shoulderMotorPosition.PARKED;
    }

    public void resetShoulder(){
        DcMotor.RunMode runMode = lshmotor.getMode();
        lshmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rshmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lshmotor.setMode(runMode);
        rshmotor.setMode(runMode);

    }

    public void runShoulderToLevel(double power){
        rshmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lshmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runShoulderToLevelState == true){
            lshmotor.setPower(power);
            rshmotor.setPower(power);
            runShoulderToLevelState = false;
        } else{
            lshmotor.setPower(0.0);
            rshmotor.setPower(0.0);
        }
    }

    public void turnShoulderBrakeModeOn() {
        lshmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rshmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void turnShoulderBrakeModeOff() {
        lshmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rshmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    //gets the position of the shoulder
    public int getShoulderPositionCount() {
        return rshmotor.getCurrentPosition();
    }

    //Sets shoulder position to ground junction
    public void moveToShoulderGroundJunction() {
        turnShoulderBrakeModeOn();
        lshmotor.setTargetPosition(GROUND_JUNCTION + baselineEncoderCount);
        rshmotor.setTargetPosition(GROUND_JUNCTION + baselineEncoderCount);
        runShoulderToLevelState = true;
        previousShoulderPosition = currentShoulderPosition;
        currentShoulderPosition = shoulderMotorPosition.GROUND_JUNCTION;
    }

    //Sets shoulder position to low junction
    public void moveToShoulderLowJunction() {
        turnShoulderBrakeModeOn();
        lshmotor.setTargetPosition(LOW_JUNCTION + baselineEncoderCount);
        rshmotor.setTargetPosition(LOW_JUNCTION + baselineEncoderCount);
        runShoulderToLevelState = true;
        previousShoulderPosition = currentShoulderPosition;
        currentShoulderPosition = shoulderMotorPosition.LOW_JUNCTION;
    }

    //Sets shoulder position or mid junction
    public void moveToShoulderMidJunction() {
        turnShoulderBrakeModeOn();
        rshmotor.setTargetPosition(MEDIUM_JUNCTION + baselineEncoderCount);
        lshmotor.setTargetPosition(MEDIUM_JUNCTION + baselineEncoderCount);
        runShoulderToLevelState = true;
        previousShoulderPosition = currentShoulderPosition;
        currentShoulderPosition = shoulderMotorPosition.MEDIUM_JUNCTION;
    }

    //Sets shoulder position to high junction
    public void moveToShoulderHighJunction() {
        turnShoulderBrakeModeOn();
        rshmotor.setTargetPosition(HIGH_JUNCTION + baselineEncoderCount);
        lshmotor.setTargetPosition(HIGH_JUNCTION + baselineEncoderCount);
        runShoulderToLevelState = true;
        previousShoulderPosition = currentShoulderPosition;
        currentShoulderPosition = shoulderMotorPosition.HIGH_JUNCTION;
    }


    public void lowerShoulder(double leftTriggerAmount) {
        shoulderDeltaCount = (int) (Math.pow((leftTriggerAmount * 1.25 - 0.25), 3) * MAX_DELTA);
        if (shoulderCurrentShoulderPositionCount > minPosition){
                //&& shoulderCurrentShoulderPositionCount >= GROUND_JUNCTION + shoulderDeltaCount)
            turnShoulderBrakeModeOn();
            shoulderCurrentShoulderPositionCount = shoulderCurrentShoulderPositionCount - shoulderDeltaCount;
            rshmotor.setTargetPosition(shoulderCurrentShoulderPositionCount);
            lshmotor.setTargetPosition(shoulderCurrentShoulderPositionCount);
            runShoulderToLevelState = true;
        }
    }

    public void raiseShoulder(double rightTriggerAmount) {
        shoulderDeltaCount = (int) (Math.pow((rightTriggerAmount * 1.25 - 0.25), 3) * MAX_DELTA);

        if (shoulderCurrentShoulderPositionCount < maxPosition && shoulderCurrentShoulderPositionCount <= GROUND_JUNCTION - shoulderDeltaCount){
            turnShoulderBrakeModeOn();
            shoulderCurrentShoulderPositionCount = shoulderCurrentShoulderPositionCount + shoulderDeltaCount;
            rshmotor.setTargetPosition(shoulderCurrentShoulderPositionCount);
            lshmotor.setTargetPosition(shoulderCurrentShoulderPositionCount);
            runShoulderToLevelState = true;
        }
    }




}
