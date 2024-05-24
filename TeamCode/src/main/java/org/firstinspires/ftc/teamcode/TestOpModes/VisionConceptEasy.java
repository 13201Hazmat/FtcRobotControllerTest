package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionConceptEasy {
    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    //Vision parameters
    private TfodProcessor tfod;
    private VisionPortal visionPortal;

    public HardwareMap hardwareMap;
    public Telemetry telemetry;

    public VisionConceptEasy(HardwareMap hardwareMap, Telemetry telemetry){
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
    }

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void initTfod() {

        // Create the TensorFlow processor the easy way.
        tfod = TfodProcessor.easyCreateWithDefaults();

        // Create the vision portal the easy way.
        if (USE_WEBCAM) {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    hardwareMap.get(WebcamName.class, "Webcam 1"), tfod);
        } else {
            visionPortal = VisionPortal.easyCreateWithDefaults(
                    BuiltinCameraDirection.BACK, tfod);
        }

    }   // end method initTfod()

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void runTfodTensorFlow() {

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        //Camera placed between Left and Right Spike Mark on RED_LEFT and BLUE_LEFT If pixel not visible, assume Right spike Mark
        if (GameField.startPosition == GameField.START_POSITION.RED_LEFT ||
                GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
        } else { //RED_RIGHT or BLUE_RIGHT
            identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
        }

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (GameField.startPosition == GameField.START_POSITION.RED_LEFT ||
                    GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
                if (recognition.getLabel() == "Pixel") {
                    if (x < 100) {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    } else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    }
                }
            } else { //RED_RIGHT or BLUE_RIGHT
                if (recognition.getLabel() == "Pixel") {
                    if (x < 100) {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    } else {
                        identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    }
                }
            }

        }   // end for() loop

    }   // end method runTfodTensorFlow()

}
