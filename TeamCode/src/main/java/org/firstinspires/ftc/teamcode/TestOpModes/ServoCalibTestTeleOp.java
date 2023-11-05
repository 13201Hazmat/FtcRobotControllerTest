package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoCalibTestTeleOp", group = "Testing")
public class ServoCalibTestTeleOp extends LinearOpMode{
    Servo masterServo;
    //double servoSetPosition;
    double servoCurrentPosition;

    public void runOpMode() throws InterruptedException {
        masterServo = hardwareMap.get(Servo.class, "launcher_servo");
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
