package org.firstinspires.ftc.teamcode.GameOpModes;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

public class AutoCorrectPosition extends Thread{

    public TrajectorySequence trajectoryCorrection;
    public Pose2d currentPose;
    public DriveTrain driveTrain;
    public LinearOpMode autoOpMode;
    public boolean exit;
    public AutoCorrectPosition (DriveTrain driveTrain, LinearOpMode autoOpMode) {
        this.autoOpMode = autoOpMode;
        this.driveTrain = driveTrain;
    }

    public void run() {
        currentPose = GameField.CurrentPose;
        //telemetry.addData("Current Pose", currentPose);
        while (autoOpMode.opModeIsActive() && !autoOpMode.isStopRequested() && !exit) {
            //if(!currentPose.equals(driveTrain.getPoseEstimate())) {
            if (isNotEqual(currentPose, driveTrain.getPoseEstimate())) {
                trajectoryCorrection = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(currentPose)
                        .build();
                driveTrain.followTrajectorySequence(trajectoryCorrection);
            }
        }
    }

    public void exit(){
        exit = true;
    }

    public boolean isNotEqual(Pose2d pose1, Pose2d pose2) {
        return (pose1.getX() - pose2.getX() > 0.1 ||
            pose1.getY() - pose2.getY() > 0.1 ||
            pose1.getHeading() - pose2.getHeading() > Math.toRadians(0.1));
    }
}
