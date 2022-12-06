package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@TeleOp(name = "Test Vision", group = "Concept")
public class TestVision extends LinearOpMode {

    public Vision vision;
    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap); //create instance of vision object
        vision.activateVuforiaTensorFlow();
        waitForStart();
        while (opModeIsActive() && !isStopRequested()) {
            GameField.VISION_IDENTIFIED_TARGET visionIdentifiedTarget = vision.runVuforiaTensorFlow();
            telemetry.addData("Detected Label", vision.detectedLabel);
            telemetry.addData("Detection Confidence", "%.2f", vision.detectionConfidence);

            telemetry.addData("Vision Objected Detected: ", visionIdentifiedTarget);
            telemetry.update();//display identified label
        }
        vision.deactivateVuforiaTensorFlow();
    }
}
