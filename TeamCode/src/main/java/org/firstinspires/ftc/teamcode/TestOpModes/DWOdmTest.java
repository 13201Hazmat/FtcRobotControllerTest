package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "DWOdom", group = "Testing")
@Disabled
public class DWOdmTest extends LinearOpMode {
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    @Override
    public void runOpMode() throws InterruptedException {
        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder_leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder_leftFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder_rightFront"));
        waitForStart();
            while (opModeIsActive()) {
                telemetry.addData("leftFront Current Position: ", leftEncoder.getCurrentPosition());
                telemetry.addData("rightFront Current Position: ", rightEncoder.getCurrentPosition());
                telemetry.addData("leftRear Current Position: ", frontEncoder.getCurrentPosition());
                telemetry.update();
            }
    }
}