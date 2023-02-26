package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "MotorTeleOp", group = "Testing")
public class MotorTeleOp extends LinearOpMode {
    public DcMotorEx motor;
    //double servoSetPosition;

    int motorCurrentPosition;



    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.get(DcMotorEx.class, "outtake_motor");
        motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        motor.setPositionPIDFCoefficients(10.0); //5
        motor.setDirection(DcMotorEx.Direction.FORWARD);
        waitForStart();
        while (opModeIsActive()) {
            motorCurrentPosition = motor.getCurrentPosition();

            if(gp1GetDpad_downPress()){
                motor.setTargetPosition(motorCurrentPosition - 50);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }
            if(gp1GetDpad_upPress()) {
                motor.setTargetPosition(motorCurrentPosition + 50);
                motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                motor.setPower(1);
            }

            telemetry.addData("Current Position", motorCurrentPosition);
            telemetry.update();
        }
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