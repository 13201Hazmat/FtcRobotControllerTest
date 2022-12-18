package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.firstinspires.ftc.teamcode.AadiGeometry.AadiPose;
import org.firstinspires.ftc.teamcode.SubSystems.Hand;

/**
 * Static Class to define Gamefield Vector positions.
 * These are used in start Position estimates and in automatic targetic.
 *
 * The static class also has PosStorage defined, to pass the last position in autonomous mode
 * to following TeleOp mode
 */
public class GameField {
    // Declare a target vector you'd like your bot to align with
    // Can be any x/y coordinate of your choosing
    public static final Vector2d ORIGIN = new Vector2d(0,0);
    public static final Pose2d ORIGINPOSE = new Pose2d(0,0,Math.toRadians(0));

    public enum DEBUG_LEVEL{
        NONE,
        MINIMUM,
        MAXIMUM
    }
    public static DEBUG_LEVEL debugLevel = DEBUG_LEVEL.MAXIMUM;

    public enum OP_MODE_RUNNING {
        HAZMAT_TELEOP,
        HAZMAT_AUTONOMOUS
    }
    public static OP_MODE_RUNNING opModeRunning = OP_MODE_RUNNING.HAZMAT_AUTONOMOUS;

    //Define and declare Playing Alliance
    public enum PLAYING_ALLIANCE{
        RED_ALLIANCE,
        BLUE_ALLIANCE,
    }
    public static PLAYING_ALLIANCE playingAlliance = PLAYING_ALLIANCE.BLUE_ALLIANCE;

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d();

    public enum VISION_IDENTIFIED_TARGET {
        LOCATION1,
        LOCATION2,
        LOCATION3
    }

    public static AadiPose PRESET_PICKUP_A = new AadiPose(735,85, Hand.WRIST_STATE.WRIST_LEVEL,1308); //Pickup
    public static AadiPose PRESET_MEDIUM_JUNCTION_B = new AadiPose(283,819, Hand.WRIST_STATE.WRIST_UP,2354); //Medium Junction
    public static AadiPose PRESET_LOW_JUNCTION_X = new AadiPose(0,318, Hand.WRIST_STATE.WRIST_UP,1665); //Low Junction
    public static AadiPose PRESET_HIGH_JUNCTION_Y = new AadiPose(1250,900, Hand.WRIST_STATE.WRIST_UP,350); //High Junction

}
