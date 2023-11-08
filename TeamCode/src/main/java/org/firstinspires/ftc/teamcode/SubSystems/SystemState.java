package org.firstinspires.ftc.teamcode.SubSystems;
import org.firstinspires.ftc.teamcode.AadiGeometry.AadiVector;
public class SystemState {
    public static OuttakeArm.OUTTAKE_ARM_STATE OuttakeArmState = OuttakeArm.OUTTAKE_ARM_STATE.TRANSFER;
    public static OuttakeArm.OUTTAKE_WRIST_STATE OuttakeWristState = OuttakeArm.OUTTAKE_WRIST_STATE.WRIST_TRANSFER;
    public static OuttakeSlides.OUTTAKE_SLIDE_STATE OuttakeSlideState = OuttakeSlides.OUTTAKE_SLIDE_STATE.MIN_RETRACTED;

    public static double ArmRadians = 0;
    public static double WristRadians = 0;
    public static double SlidesExtensionMM = 0; //in millimeters

    public static final double SLIDES_LENGTH_TO_MM_RATIO = 1;


}
