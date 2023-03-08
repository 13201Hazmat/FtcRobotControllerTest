package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "ServoTeleOpArm Calib", group = "Testing")
public class ServoTeleOpArmCalib extends LinearOpMode {
    Servo leftServo, rightServo, intakeWristServoLeft;
    //double servoSetPosition;
    double leftServoCurrentPosition, rightServoCurrentPosition, intakeWristServoLeftCurrentPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        leftServo = hardwareMap.get(Servo.class, "intake_arm_left");
        rightServo = hardwareMap.get(Servo.class, "intake_arm_right");
        intakeWristServoLeft = hardwareMap.get(Servo.class, "intake_wrist_left");


        leftServo.setPosition(0.51);
        rightServo.setPosition(0.5);
        intakeWristServoLeft.setPosition(0.5);

        waitForStart();
            while (opModeIsActive()) {
                leftServoCurrentPosition = leftServo.getPosition();
                rightServoCurrentPosition = rightServo.getPosition();
                intakeWristServoLeftCurrentPosition = intakeWristServoLeft.getPosition();
                if(gp1GetDpad_downPress()){
                    if(leftServoCurrentPosition > 0) { //0
                        leftServo.setPosition(leftServoCurrentPosition - 0.005);
                        rightServo.setPosition(rightServoCurrentPosition + 0.005);
                    }
                }
                if(gp1GetDpad_upPress()){
                    if(leftServoCurrentPosition < 1) {
                        leftServo.setPosition(leftServoCurrentPosition + 0.005);
                        rightServo.setPosition(rightServoCurrentPosition - 0.005);
                    }
                }

                if(gp1GetXPress()){
                    if(intakeWristServoLeftCurrentPosition > 0) { //0
                        intakeWristServoLeft.setPosition(intakeWristServoLeftCurrentPosition - 0.01);
                    }
                }
                if(gp1GetAPress()){
                    if(intakeWristServoLeftCurrentPosition < 1){ //0
                        intakeWristServoLeft.setPosition(intakeWristServoLeftCurrentPosition + 0.01);
                    }
                }

                telemetry.addData("Left Servo Current Position: ", leftServo.getPosition());
                telemetry.addData("Right Servo Current Position: ", rightServo.getPosition());
                telemetry.addData("Intake Servo Current Position: ", intakeWristServoLeft.getPosition());

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

    boolean gp1ALast;
    public boolean gp1GetAPress() {
        boolean isPressedA;
        isPressedA = false;
        if (!gp1ALast && gamepad1.a) {
            isPressedA = true;
        }
        gp1ALast = gamepad1.a;
        return isPressedA;
    }

    boolean gp1XLast;
    public boolean gp1GetXPress() {
        boolean isPressedX;
        isPressedX = false;
        if (!gp1XLast && gamepad1.x) {
            isPressedX = true;
        }
        gp1XLast = gamepad1.x;
        return isPressedX;
    }

}