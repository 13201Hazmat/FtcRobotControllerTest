package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;

/**
 * Static Class to define Gamefield Vector positions.
 * These are used in start Position estimates and in automatic targetic.
 *
 * The static class also has PosStorage defined, to pass the last position in autonomous mode
 * to following TeleOp mode
 */
public class GameField {

    //Define and declare Robot Starting Locations
    public enum START_POSITION{
        BLUE_LEFT,
        BLUE_RIGHT,
        RED_LEFT,
        RED_RIGHT
    }
    public static START_POSITION startPosition;

    public enum PLAYING_ALLIANCE{
        BLUE_ALLIANCE,
        RED_ALLIANCE
    }
    public static  PLAYING_ALLIANCE playingAlliance;

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

    //Static fields to pass Pos from Autonomous to TeleOp
    public static boolean poseSetInAutonomous = false;
    public static Pose2d currentPose = new Pose2d(0,0,0);

    public static boolean testVision = false;

    public static Pose2d CurrentPose;

}
