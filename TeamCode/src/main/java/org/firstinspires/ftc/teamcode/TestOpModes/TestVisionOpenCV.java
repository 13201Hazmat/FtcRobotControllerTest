package org.firstinspires.ftc.teamcode.TestOpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.SubSystems.VisionOpenCV;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
public class TestVisionOpenCV extends OpMode{
    private VisionOpenCV visionOpenCV;

    private VisionPortal visionPortal;
    @Override

    public void init() {
        visionOpenCV = new VisionOpenCV(hardwareMap, telemetry, "Webcam 1");
        /*visionPortal = VisionPortal.easyCreateWithDefaults(
                hardwareMap.get(WebcamName.class, "Webcam 1"), visionOpenCV);*/
    }

    @Override
    public void init_loop() {
        telemetry.addData("Identified", visionOpenCV.getSelection());
        telemetry.addData("SatLeftOfCameraMid", visionOpenCV.satRectLeftOfCameraMid);

        telemetry.addData("SatRightOfCameraMid", visionOpenCV.satRectRightOfCameraMid);
        telemetry.addData("SatRectNone", visionOpenCV.satRectNone);
        telemetry.update();
    }

    @Override
    public void start() {
            visionPortal.stopStreaming();
        }

    @Override
    public void loop() {
        telemetry.addData("Identified", visionOpenCV.getSelection());
        telemetry.addData("SatLeftOfCameraMid", visionOpenCV.satRectLeftOfCameraMid);

        telemetry.addData("SatRightOfCameraMid", visionOpenCV.satRectRightOfCameraMid);
        telemetry.addData("SatRectNone", visionOpenCV.satRectNone);
        telemetry.update();
    }
}
