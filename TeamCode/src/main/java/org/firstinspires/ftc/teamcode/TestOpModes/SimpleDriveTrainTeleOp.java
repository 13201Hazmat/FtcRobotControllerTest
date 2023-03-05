package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.util.Encoder;

@TeleOp(name = "DriveTrainTestTeleOp", group = "Testing")
@Disabled
public class SimpleDriveTrainTeleOp extends LinearOpMode {
    public DcMotorEx leftFront, leftRear, rightRear, rightFront;
    private Encoder leftEncoder, rightEncoder, frontEncoder;

    double leftFrontpower, leftRearpower, rightRearpower, rightFrontpower;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotorEx.class, "rightEncoder_leftFront");
        leftRear = hardwareMap.get(DcMotorEx.class, "leftEncoder_leftRear");
        rightRear = hardwareMap.get(DcMotorEx.class, "rightRear");
        rightFront = hardwareMap.get(DcMotorEx.class, "frontEncoder_rightFront");

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "leftEncoder_leftRear"));
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "rightEncoder_leftFront"));
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "frontEncoder_rightFront"));

        int leftStart, rightStart, frontStart;

        leftStart = leftEncoder.getCurrentPosition();
        rightStart = rightEncoder.getCurrentPosition();
        frontStart = frontEncoder.getCurrentPosition();

        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setDirection(DcMotor.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            if(gamepad1.dpad_down){
                leftFront.setPower(-gamepad1.right_trigger);
                leftRear.setPower(-gamepad1.right_trigger);
                rightRear.setPower(-gamepad1.right_trigger);
                rightFront.setPower(-gamepad1.right_trigger);
            } else if(gamepad1.dpad_up) {
                leftFront.setPower(gamepad1.right_trigger);
                leftRear.setPower(gamepad1.right_trigger);
                rightRear.setPower(gamepad1.right_trigger);
                rightFront.setPower(gamepad1.right_trigger);
            } else {
                leftFront.setPower(0);
                leftRear.setPower(0);
                rightRear.setPower(0);
                rightFront.setPower(0);
            }

            telemetry.addData("leftEncoder Current Position: ", leftEncoder.getCurrentPosition()-leftStart);
            telemetry.addData("rightEncoder Current Position: ", rightEncoder.getCurrentPosition()-rightStart);
            telemetry.addData("frontEncoder Current Position: ", frontEncoder.getCurrentPosition()-frontStart);
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