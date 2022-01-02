package org.firstinspires.ftc.teamcode.TestingOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Calibrate Motor Position", group = "Calibration")
public class CalibrateMotorPosition extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public DcMotorEx motorToCalibrate = null;
    public double motorPower = 0.5;
    public boolean runMotorToLevelState = false;
    public int motorPositionCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        //motorToCalibrate =  hardwareMap.get(DcMotorEx.class, "motorToCalibrate");;
        //motorToCalibrate =  hardwareMap.get(DcMotorEx.class, "elevator_motor");
        motorToCalibrate =  hardwareMap.get(DcMotorEx.class, "major_arm_motor");

        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        initMotorToCalibrate();
        resetMotorToCalibrate();

        /* Wait for Start or Stop Button to be pressed */
        waitForStart();

        /* If Stop is pressed, exit OpMode */
        if (isStopRequested()) return;

        /*If Start is pressed, enter loop and exit only when Stop is pressed */
        while (!isStopRequested()) {

            if(DEBUG_FLAG) {
                printDebugMessages();
                telemetry.update();
            }

            while (opModeIsActive()) {
                gamepadTestController.runByGamepadControl();

                if (gamepadTestController.getDpad_downPress()){
                   if (motorPower >-1.0) motorPower -=0.05;
                }
                if (gamepadTestController.getDpad_upPress()) {
                    if (motorPower <1.0) motorPower +=0.05;
                }

                if (gamepadTestController.getRightBumperPress()){
                    motorPositionCount +=50;
                }

                if (gamepadTestController.getLeftBumperPress()){
                    motorPositionCount -=50;
                }

                moveMotorLevelToPosition(motorPositionCount);
                runMotorToLevel(motorPower);

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
    }


    /**
     * Initialization for the Elevator
     */
    public void initMotorToCalibrate(){
        motorToCalibrate.setPositionPIDFCoefficients(5.0);
        motorToCalibrate.setDirection(DcMotorSimple.Direction.FORWARD);
        motorToCalibrate.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetMotorToCalibrate();
        turnMotorBrakeModeOff();
    }

    /**
     * Reset Elevator Encoder
     */
    public void resetMotorToCalibrate(){
        DcMotor.RunMode runMode = motorToCalibrate.getMode();
        motorToCalibrate.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorToCalibrate.setMode(runMode);
    }

    /**
     * Method to set Elevator brake mode to ON when Zero (0.0) power is applied. <BR>
     * To be used when arm is above groundlevel
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnMotorBrakeModeOn(){
        motorToCalibrate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /**
     * Method to set Elevtor brake mode to OFF when Zero (0.0) power is applied. <BR>
     * To be used when arm is on groundlevel or blockLevel[0]
     * setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
     */
    public void turnMotorBrakeModeOff(){
        motorToCalibrate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }

    /**
     * Method to run motor to set to the set position
     */
    public void runMotorToLevel(double power){
        motorToCalibrate.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (runMotorToLevelState == true || motorToCalibrate.isBusy() == true){
            motorToCalibrate.setPower(power);
            runMotorToLevelState = false;
        } else {
            motorToCalibrate.setPower(0.0);
        }
    }

    /**
     * Move Elevator to Level 0
     */
    public void moveMotorLevelToPosition(int motorPositionCount) {
        turnMotorBrakeModeOn();
        motorToCalibrate.setTargetPosition(motorPositionCount);
        runMotorToLevelState = true;
    }




    /**
     * Safe method to wait so that stop button is also not missed
     * @param time time in ms to wait
     */
    public void safeWait(double time){
        ElapsedTime timer = new ElapsedTime(MILLISECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time){
            //Wait
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_FLAG is : ", DEBUG_FLAG);

        //****** Drive debug ******
        telemetry.addData("Battery Power : ", driveTrain.getBatteryVoltage(hardwareMap));

        telemetry.addData("Motor Mode : ", motorToCalibrate.getMode());
        telemetry.addData("Motor Direction : ", motorToCalibrate.getDirection());
        telemetry.addData("Motor Power : ", motorToCalibrate.getPower());
        telemetry.addData("Motor Position Set : ", motorPositionCount);
        telemetry.addData("Motor Position Actual : ", motorToCalibrate.getTargetPosition());
        //Add logic for debug print Logic

        telemetry.update();

    }
}
