package org.firstinspires.ftc.teamcode.SubSystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.camera.calibration.CameraCalibration;
import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;


public class VisionOpenCV implements VisionProcessor {
    public Rect rectLeftOfCameraMid = new Rect(10, 40, 150, 240);
    public Rect rectRightOfCameraMid = new Rect(160, 40, 470, 160);
    CameraSelectedAroundMid selectionAroundMid = CameraSelectedAroundMid.NONE;

    public enum IDENTIFIED_SPIKE_MARK_LOCATION {
        LEFT,
        MIDDLE,
        RIGHT
    }
    public static IDENTIFIED_SPIKE_MARK_LOCATION identifiedSpikeMarkLocation ;

    public VisionPortal visionPortal;


    Mat submat = new Mat();
    Mat hsvMat = new Mat();

    public double satRectLeftOfCameraMid, satRectRightOfCameraMid;
    public double satRectNone = 40.0;

    public VisionOpenCV(HardwareMap hardwareMap){
        visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), this);
    }

    @Override
    public void init(int width, int height, CameraCalibration calibration) {
    }


    @Override
    public Object processFrame(Mat frame, long captureTimeNanos) {
        Imgproc.cvtColor(frame, hsvMat, Imgproc.COLOR_RGB2HSV);
        satRectLeftOfCameraMid = getAvgSaturation(hsvMat, rectLeftOfCameraMid);
        satRectRightOfCameraMid = getAvgSaturation(hsvMat, rectRightOfCameraMid);

        if ((satRectLeftOfCameraMid > satRectRightOfCameraMid) && (satRectLeftOfCameraMid > satRectNone)) {
            return CameraSelectedAroundMid.LEFT_OF_CAMERA_MID;
        } else if ((satRectRightOfCameraMid > satRectLeftOfCameraMid) && (satRectRightOfCameraMid > satRectNone)) {
            return CameraSelectedAroundMid.RIGHT_OF_CAMERA_MID;
        }
        return CameraSelectedAroundMid.NONE;
    }

    protected double getAvgSaturation(Mat input, Rect rect) {
        submat = input.submat(rect);
        Scalar color = Core.mean(submat);
        return color.val[1];
    }

    private android.graphics.Rect makeGraphicsRect(Rect rect, float scaleBmpPxToCanvasPx) {
        int left = Math.round(rect.x * scaleBmpPxToCanvasPx);
        int top = Math.round(rect.y * scaleBmpPxToCanvasPx);
        int right = left + Math.round(rect.width * scaleBmpPxToCanvasPx);
        int bottom = top + Math.round(rect.height * scaleBmpPxToCanvasPx);

        return new android.graphics.Rect(left, top, right, bottom);
    }

    @Override
    public void onDrawFrame(Canvas canvas, int onscreenWidth, int onscreenHeight, float scaleBmpPxToCanvasPx, float scaleCanvasDensity, Object userContext) {
        Paint selectedPaint = new Paint();
        selectedPaint.setColor(Color.RED);
        selectedPaint.setStyle(Paint.Style.STROKE);
        selectedPaint.setStrokeWidth(scaleCanvasDensity * 4);

        Paint nonSelectedPaint = new Paint(selectedPaint);
        nonSelectedPaint.setColor(Color.GREEN);

        android.graphics.Rect drawRectangleLeft = makeGraphicsRect(rectLeftOfCameraMid, scaleBmpPxToCanvasPx);
        android.graphics.Rect drawRectangleMiddle = makeGraphicsRect(rectRightOfCameraMid, scaleBmpPxToCanvasPx);



        selectionAroundMid = (CameraSelectedAroundMid) userContext;
        switch (selectionAroundMid) {
            case LEFT_OF_CAMERA_MID:
                canvas.drawRect(drawRectangleLeft, selectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
            case RIGHT_OF_CAMERA_MID:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, selectedPaint);
                break;
            case NONE:
                canvas.drawRect(drawRectangleLeft, nonSelectedPaint);
                canvas.drawRect(drawRectangleMiddle, nonSelectedPaint);
                break;
        }
    }

    public IDENTIFIED_SPIKE_MARK_LOCATION getSelection() {
        if (GameField.startPosition == GameField.START_POSITION.RED_LEFT ||
                GameField.startPosition == GameField.START_POSITION.BLUE_LEFT) {
            switch (selectionAroundMid) {
                case NONE:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    break;
                case LEFT_OF_CAMERA_MID:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    break;
                case RIGHT_OF_CAMERA_MID:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    break;
            }
        } else { //RED_RIGHT or BLUE_RIGHT
            switch (selectionAroundMid) {
                case NONE:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.LEFT;
                    break;
                case LEFT_OF_CAMERA_MID:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.MIDDLE;
                    break;
                case RIGHT_OF_CAMERA_MID:
                    identifiedSpikeMarkLocation = IDENTIFIED_SPIKE_MARK_LOCATION.RIGHT;
                    break;
            }
        }
        return identifiedSpikeMarkLocation;
    }

    public enum CameraSelectedAroundMid {
        NONE,
        LEFT_OF_CAMERA_MID,
        RIGHT_OF_CAMERA_MID
    }

}
