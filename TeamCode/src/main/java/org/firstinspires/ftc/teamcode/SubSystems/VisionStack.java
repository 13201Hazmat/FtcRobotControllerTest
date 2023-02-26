package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;

import java.util.List;

public class VisionStack {
    public enum VISION_STACK_STATE {
        TFOD_INIT,
        TFOD_ACTIVE,
        TFOD_RUNNING,
        INACTIVE
    }
    public VisionStack.VISION_STACK_STATE visionStackState = VisionStack.VISION_STACK_STATE.INACTIVE;

    public enum ACTIVE_WEBCAM{
        WEBCAM1,
    }


    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AZME4Mr/////AAABmY+MAyxxT0IileR7JBqaAPsxN2XNYlaGBtEjYaHOlVVTqPQf7NH9eIrosYKKQHPGEXLtJUsdMwZ9e3EXBfy6arulcLPvdpW9bqAB2F2MJJXo35lLA096l/t/LQTi+etVso0Xc5RYkTVSIP3YABp1TeOaF8lCSpjVhPIVW3l/c/XlrnEMPhJk9IgqMEp4P/ifqAqMMMUAIKPEqIrXIv79TvAfdIJig46gfQGaQl5tFHr3nmvMbh/LhFrh5AWAy3B/93cCkOszmYkdHxZStbNB5lMdkTnf3sCnYbQY4jviorfhYrAkqHWH6vNOB9lUt8dOSeHsDtlk33e/6xQgOCNYFN80anYMp82JNDBFX3oyGliV";


    // Class Members
    //private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    //private VuforiaTrackables targets   = null ;

    /**
     * This is the webcam we are to use. As with other hardware devices such as motors and
     * servos, this device is identified using the robot configuration tool in the FTC application.
     */
    WebcamName webcamName = null;

    public  boolean targetVisible = false;
    private float phoneXRotate    = 0;
    private float phoneYRotate    = 0;
    private float phoneZRotate    = 0;

    public VuforiaLocalizer.Parameters parameters;

    public List<VuforiaTrackable> allTrackables;

    //Tensor Flow parameters
    /* Note: This sample uses the all-objects Tensor Flow model (PowerPlay.tflite), which contains
     * the following 3 detectable objects
     *  0: Bolt,
     *  1: Bulb,
     *  2: Panel,
     */
    //private static final String TFOD_MODEL_ASSET = "PowerPlay.tflite";
    private static final String TFOD_MODEL_ASSET = "PowerPlayConeStack.tflite";
    // private static final String TFOD_MODEL_FILE  = "/sdcard/FIRST/tflitemodels/CustomTeamModel.tflite";


    private static final String[] LABELS = {
            /*"1 Bolt",
            "2 Bulb",
            "3 Panel",*/
            "Blue Stack",
            "Red Stack"
    };



    public String detectedLabel = "None";
    public float detectedLabelLeft;
    public double detectionConfidence;

    private TFObjectDetector tfod;
    private Recognition recognition;
    public GameField.VISION_IDENTIFIED_TARGET visionIdentifiedTarget = GameField.VISION_IDENTIFIED_TARGET.LOCATION1;

    Vision.ACTIVE_WEBCAM activeWebcam;
    /**
     * Initialize the Vuforia localization engine.
     */
    public VisionStack(HardwareMap hardwareMap) {
        activeWebcam = Vision.ACTIVE_WEBCAM.WEBCAM1;

        if (activeWebcam == Vision.ACTIVE_WEBCAM.WEBCAM1) {
            webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        }
        /*} else { //TODO: Uncomment if using 2 cameras;
            webcamName = hardwareMap.get(WebcamName.class, "Webcam2");
        }*/


        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         * We can pass Vuforia the handle to a camera preview resource (on the RC phone);
         * If no camera monitor is desired, use the parameter-less constructor instead (commented out below).
         */
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;

        /**
         * We also indicate which camera on the RC we wish to use.
         */
        parameters.cameraName = webcamName;

        // Make sure extended tracking is disabled for this example.
        parameters.useExtendedTracking = false;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
        initTfod(hardwareMap);
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod(HardwareMap hardwareMap) {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.70f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 300;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        // Use loadModelFromAsset() if the TF Model is built in as an asset by Android Studio
        // Use loadModelFromFile() if you have downloaded a custom team model to the Robot Controller's FLASH.
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
        // tfod.loadModelFromFile(TFOD_MODEL_FILE, LABELS);

    }

    /**
     * Activate Vuforia Tensor Flow to determine target zone
     * This is to be done at Init in Autonomous mode
     */
    public void activateVuforiaTensorFlow(){
        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
            visionStackState = VisionStack.VISION_STACK_STATE.TFOD_ACTIVE;

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(1.75, 16.0/9.0);
            tfod.setZoom(1, 16.0/9.0);
            recognition = tfod.getUpdatedRecognitions().get(0);
        }
    }

    /**
     * Run Tensor flow algorithm.
     * This is to be run till the play button is pressed.. the last target zone identified is returned.
     * @return
     */
    public float runVuforiaTensorFlow() {
        visionStackState = VisionStack.VISION_STACK_STATE.TFOD_RUNNING;
        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            recognition = tfod.getUpdatedRecognitions().get(0);


            if (recognition != null) {
                //telemetry.addData("# Object Detected", updatedRecognitions.size());
                /*for (Recognition recognition : recognition) {
                    // check label to see which target zone to go after.
                    detectedLabel = recognition.getLabel();
                    /*switch(detectedLabel){
                        //case "Blue Stack":
                        case "Blue Stack":
                            visionIdentifiedTarget = GameField.VISION_IDENTIFIED_TARGET.LOCATION1;
                            break;
                        //case "Red Stack":
                        case "Red Stack":*/
                //Adjusted to match to recogonition by Hazmat Model
                if (recognition.getLabel() == "Red Stack") {
                    detectedLabelLeft = recognition.getLeft();
                }
            }
        }
        return detectedLabelLeft;

    }

    /**
     * Stop Tensor Flow algorithm
     */
    public void deactivateVuforiaTensorFlow(){
        if (tfod != null) {
            tfod.shutdown();
            visionStackState = VisionStack.VISION_STACK_STATE.INACTIVE;
        }
    }
}
/*Footer
        2022 GitHub, Inc.
        Footer navigation
        Terms
        Privacy
        Security
        Status
        Docs
        Contact GitHub
        Pricing
        API
        Training
        Blog
        About
        FtcRobotController/Vision.java at 2022WorldChampionships · 13201Hazmat/FtcRobotController{
}
*/