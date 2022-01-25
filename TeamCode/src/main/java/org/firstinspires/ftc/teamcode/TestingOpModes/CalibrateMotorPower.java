package org.firstinspires.ftc.teamcode.TestingOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

/**
 * Ultimate Goal TeleOp mode <BR>
 *
 *  *  This code defines the TeleOp mode is done by Hazmat Robot for Ultimate Goal.<BR>
 *
 */
@TeleOp(name = "Calibrate Motor Power", group = "Calibration")
@Disabled
public class CalibrateMotorPower extends LinearOpMode {

    public boolean DEBUG_FLAG = true;

    public GamepadTestController gamepadTestController;
    public DriveTrain driveTrain;
    public DcMotor motorToCalibrate;
    public double motorPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        /* Create Subsystem Objects*/
        driveTrain = new DriveTrain(hardwareMap);

        //motorToCalibrate = hardwareMap.dcMotor.get("motor_name"); //Change name of motor to calibrate per hardwaremap
        //motorToCalibrate = hardwareMap.dcMotor.get("elevator_motor");
        motorToCalibrate = hardwareMap.dcMotor.get("spinner_motor");

        /* Create Controllers */
        gamepadTestController = new GamepadTestController(gamepad1, driveTrain);

        /* Set Initial State of any subsystem when TeleOp is to be started*/
        motorToCalibrate.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorToCalibrate.setDirection(DcMotorSimple.Direction.FORWARD);

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

                motorToCalibrate.setPower(motorPower);

                if(DEBUG_FLAG) {
                    printDebugMessages();
                    telemetry.update();
                }

            }

        }
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
        //Add logic for debug print Logic

        telemetry.update();

    }
}
