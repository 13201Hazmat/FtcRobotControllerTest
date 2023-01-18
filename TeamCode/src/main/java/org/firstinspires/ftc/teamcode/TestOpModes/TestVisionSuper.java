package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

@TeleOp(name = "Test Vision Super", group = "Concept")

public class TestVisionSuper extends LinearOpMode {

    public VisionSuper vision;
    public VisionSuper.ACTIVE_WEBCAM activeWebcam = VisionSuper.ACTIVE_WEBCAM.WEBCAM1;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new VisionSuper(hardwareMap); //create instance of vision object
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
