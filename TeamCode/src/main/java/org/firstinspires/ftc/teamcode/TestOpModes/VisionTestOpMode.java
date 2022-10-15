package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.SubSystems.Vision;

@TeleOp(name = "Vision Test OpMode", group = "Concept")
public class VisionTestOpMode extends LinearOpMode {

    public HardwareMap hardwareMap;
    public Vision vision;
    public Vision.ACTIVE_WEBCAM activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;

    @Override
    public void runOpMode() throws InterruptedException {
        vision = new Vision(hardwareMap, activeWebcam); //create instance of vision object
        vision.activateVuforiaTensorFlow();
        while (opModeIsActive() && !isStopRequested()) {
            String detectedLabel = vision.runVuforiaTensorFlow();
            telemetry.addData("Vision Objected Detected: ", detectedLabel); //display identified label
        }
        vision.deactivateVuforiaTensorFlow();
    }
}
