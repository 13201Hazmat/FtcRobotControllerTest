package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "MotorCalibTestTeleOp", group = "Testing")
public class MotorCalibTestTeleOp extends LinearOpMode{
    DcMotorEx masterMotor;
    int motorCurrentPosition;

    public void runOpMode() throws InterruptedException {
        masterMotor = hardwareMap.get(DcMotorEx.class, "climber_motor");
        masterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        masterMotor.setPositionPIDFCoefficients(10.0); //5
        masterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        waitForStart();
        while (opModeIsActive()) {
           motorCurrentPosition = masterMotor.getCurrentPosition();
            while(gp1GetDpad_up() && !isStopRequested()){
                masterMotor.setTargetPosition(motorCurrentPosition + 10);
                masterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                masterMotor.setPower(0.5);
            }
            while(gp1GetDpad_down() && !isStopRequested()){
                masterMotor.setTargetPosition(motorCurrentPosition - 10);
                masterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
                masterMotor.setPower(0.5);
            }
            masterMotor.setPower(0);
            telemetry.addData("Motor Current Position: ", masterMotor.getCurrentPosition());
            telemetry.update();
        }
    }
    public boolean gp1GetDpad_up(){
        return gamepad1.dpad_up;
    }
    public boolean gp1GetDpad_down(){
        return gamepad1.dpad_down;
    }
    boolean gp1Dpad_upLast;
    public boolean gp1GetDpad_upPress() {
        boolean isPressedDpad_up;
        isPressedDpad_up = false;
        if (!gp1Dpad_upLast && gamepad1.dpad_up) {
            isPressedDpad_up = true;
        }
        gp1Dpad_upLast = gamepad1.dpad_up;
        return isPressedDpad_up;
    }

    boolean gp1Dpad_downLast;

    public boolean gp1GetDpad_downPress() {
        boolean isPressedDpad_down;
        isPressedDpad_down = false;
        if (!gp1Dpad_downLast && gamepad1.dpad_down) {
            isPressedDpad_down = true;
        }
        gp1Dpad_downLast = gamepad1.dpad_down;
        return isPressedDpad_down;
    }
}
