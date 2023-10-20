package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.GameOpModes.FTCWiresAutonomous;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;

public class VisionDual {

    public enum VISION_TYPE {
        WEBCAM1_APRILTAG,
        WEBCAM2_APRILTAG,
        WEBCAM1_TFOD,
        WEBCAM2_TFOD,
        NONE
    }
    public VISION_TYPE visionType = VISION_TYPE.WEBCAM1_APRILTAG;

    public enum ACTIVE_WEBCAM{
        WEBCAM1,
        WEBCAM2,
        NONE
    }
    public ACTIVE_WEBCAM activeWebcam = ACTIVE_WEBCAM.WEBCAM1;

    private static final boolean USE_WEBCAM = true;  // true for webcam, false for phone camera

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public AprilTagProcessor aprilTag;

    /**
     * The variable to store our instance of the TensorFlow Object Detection processor.
     */
    public TfodProcessor tfod;

    /**
     * The variable to store our instance of the AprilTag processor.
     */
    public VisionPortal visionPortal;
    public WebcamName webcam1, webcam2;

    Telemetry telemetry;
    HardwareMap hardwareMap;

    // TFOD_MODEL_ASSET points to a model file stored in the project Asset location,
    // this is only used for Android Studio when using models in Assets.
    private static final String TFOD_MODEL_ASSET = "MyModelStoredAsAsset.tflite";
    // TFOD_MODEL_FILE points to a model file stored onboard the Robot Controller's storage,
    // this is used when uploading models directly to the RC using the model upload interface.
    private static final String TFOD_MODEL_FILE = "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";
    // Define the labels recognized in the model for TFOD (must be in training order!)
    private static final String[] LABELS = {
            "Pixel",
    };

    public VisionDual(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        activeWebcam = ACTIVE_WEBCAM.WEBCAM1;
    }

    public void activateAprilTag(){
        initAprilTag();

        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    public void activateDoubleVision(){
        initDoubleVision();
        // Wait for the DS start button to be touched.
        telemetry.addData("DS preview on/off", "3 dots, Camera Stream");
        telemetry.addData(">", "Touch Play to start OpMode");
        telemetry.update();
    }

    private void initAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                .addProcessor(aprilTag)
                .build();

    }   // end method initAprilTag()

    /**
     * Add telemetry about AprilTag detections.
     */
    public void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");
    }   // end method telemetryAprilTag()


    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static FTCWiresAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation = FTCWiresAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;

    /**
     * Initialize the TensorFlow Object Detection processor.
     */
    public void initTfod() {

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // With the following lines commented out, the default TfodProcessor Builder
                // will load the default model for the season. To define a custom model to load,
                // choose one of the following:
                //   Use setModelAssetName() if the custom TF Model is built in as an asset (AS only).
                //   Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                // The following default settings are available to un-comment and edit as needed to
                // set parameters for custom models.
                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
        if (USE_WEBCAM) {
            builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        } else {
            builder.setCamera(BuiltinCameraDirection.BACK);
        }

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }   // end method initTfod()

    public void runTfodTensorFlow(){
        telemetryTfod();
    }

    /**
     * Add telemetry about TensorFlow Object Detection (TFOD) recognitions.
     */
    public void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions();
        telemetry.addData("# Objects Detected", currentRecognitions.size());

        // Step through the list of recognitions and display info for each one.
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            telemetry.addData(""," ");
            telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            telemetry.addData("- Position", "%.0f / %.0f", x, y);
            telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());

            if (x < 100) {
                identifiedSpikeMarkLocation = FTCWiresAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
            } else if (x>100 && x <200) {
                identifiedSpikeMarkLocation = FTCWiresAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
            } else { //x > 200
                identifiedSpikeMarkLocation = FTCWiresAutonomous.IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
            }

            telemetry.addData("Vision identified Spike Mark location",identifiedSpikeMarkLocation);

        }   // end for() loop

    }   // end method telemetryTfod()


    private void initDoubleVision() {

        // Create the AprilTag & Tfod processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();
        tfod = new TfodProcessor.Builder()
                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                //.setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                //.setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)
                .build();

        webcam1 = hardwareMap.get(WebcamName.class, "Webcam 1");
        webcam2 = hardwareMap.get(WebcamName.class, "Webcam 2");
        CameraName switchableCamera = ClassFactory.getInstance()
                .getCameraManager().nameForSwitchableCamera(webcam1, webcam2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(switchableCamera)
                //.setCameraResolution(new Size(640, 480));
                .addProcessors(tfod,aprilTag)
                .build();

    }   // end method initAprilTag()

    public void telemetryCameraSwitching() {
        if (visionPortal.getActiveCamera().equals(webcam1)) {
            telemetry.addData("activeCamera", "Webcam 1");
            telemetry.addData("Press RightBumper", "to switch to Webcam 2");
        } else {
            telemetry.addData("activeCamera", "Webcam 2");
            telemetry.addData("Press LeftBumper", "to switch to Webcam 1");
        }

    }   // end method telemetryCameraSwitching()

    public void printDebugMessages(){
        telemetryCurrentVisionPortal();
        telemetry.addLine("=================");
    }

    public void telemetryCurrentVisionPortal(){
        telemetry.addData("Current Vision Portal:", visionType);
        switch(visionType) {
            case WEBCAM1_APRILTAG:
            case WEBCAM2_APRILTAG:
                //telemetryAprilTag(); //TODO Uncomment later
                break;
            case WEBCAM1_TFOD:
            case WEBCAM2_TFOD:
                telemetryTfod();
                break;
            case NONE:
                break;
        }
    }

    public void streamCurrentVisionPortal(){
        switch(visionType) {
            case WEBCAM1_APRILTAG:
            case WEBCAM1_TFOD:
                streamWebcam1();
                break;
            case WEBCAM2_APRILTAG:
            case WEBCAM2_TFOD:
                streamWebcam2();
                break;
            case NONE:
                break;
        }
    }

    public void switchWebcamAction(VISION_TYPE newVisionType){
        switch(newVisionType) {
            case WEBCAM1_APRILTAG:
                visionType = VISION_TYPE.WEBCAM1_APRILTAG;
                visionPortal.setProcessorEnabled(tfod,false);
                visionPortal.setProcessorEnabled(aprilTag,true);
                break;
            case WEBCAM2_APRILTAG:
                visionType = VISION_TYPE.WEBCAM2_APRILTAG;
                visionPortal.setProcessorEnabled(tfod,false);
                visionPortal.setProcessorEnabled(aprilTag,true);
                break;
            case WEBCAM1_TFOD:
                visionType = VISION_TYPE.WEBCAM1_TFOD;
                visionPortal.setProcessorEnabled(aprilTag,false);
                visionPortal.setProcessorEnabled(tfod,true);
                break;
            case WEBCAM2_TFOD:
                visionType = VISION_TYPE.WEBCAM2_TFOD;
                visionPortal.setProcessorEnabled(aprilTag,false);
                visionPortal.setProcessorEnabled(tfod,true);
                break;
            case NONE:
                visionType = VISION_TYPE.NONE;
                visionPortal.setProcessorEnabled(aprilTag,false);
                visionPortal.setProcessorEnabled(tfod,false);
                break;
        }
    }

    public void streamWebcam1(){
        activeWebcam = ACTIVE_WEBCAM.WEBCAM1;
        visionPortal.setActiveCamera(webcam1);
        visionPortal.resumeStreaming();
    }

    public void streamWebcam2(){
        activeWebcam = ACTIVE_WEBCAM.WEBCAM2;
        visionPortal.setActiveCamera(webcam2);
        visionPortal.resumeStreaming();
    }

    public void stopWebcamStreaming(){
        if (visionPortal.getCameraState() == VisionPortal.CameraState.STREAMING) {
            visionPortal.stopStreaming();
        }
        activeWebcam = ACTIVE_WEBCAM.NONE;
    }

    public void deactivateVision(){
        //visionPortal.close();
    }
}
