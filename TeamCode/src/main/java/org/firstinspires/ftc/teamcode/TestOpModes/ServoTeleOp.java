package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;

@TeleOp(name = "ServoTeleOp", group = "Testing")
public class ServoTeleOp extends LinearOpMode {
    Servo masterServo;
    //double servoSetPosition;
    double servoCurrentPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        masterServo = hardwareMap.get(Servo.class, "outtake_arm_left");
        waitForStart();
            while (opModeIsActive()) {
                servoCurrentPosition = masterServo.getPosition();
                if(gp1GetDpad_downPress()){
                    if(servoCurrentPosition > -1){ //0
                        masterServo.setPosition(servoCurrentPosition - 0.01);
                    }  else {
                        masterServo.setPosition(0);
                    }
                }
                if(gp1GetDpad_upPress()){
                    if(servoCurrentPosition < 1){
                        masterServo.setPosition(servoCurrentPosition + 0.01);
                    }  else {
                        masterServo.setPosition(1);
                    }
                }
                telemetry.addData("Servo Current Position: ", masterServo.getPosition());
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