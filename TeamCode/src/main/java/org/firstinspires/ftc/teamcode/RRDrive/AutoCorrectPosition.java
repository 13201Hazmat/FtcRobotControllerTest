package org.firstinspires.ftc.teamcode.RRDrive;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.GameOpModes.GameField;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;

public class AutoCorrectPosition extends Thread{

    //public TrajectorySequence trajectoryCorrection;
    public Pose2d currentPose;
    public DriveTrain driveTrain;
    public LinearOpMode autoOpMode;
    public boolean exit;
    public AutoCorrectPosition(DriveTrain driveTrain, LinearOpMode autoOpMode) {
        this.autoOpMode = autoOpMode;
        this.driveTrain = driveTrain;
    }

    public void run() {
        currentPose = GameField.CurrentPose;
        //telemetry.addData("Current Pose", currentPose);
        while (autoOpMode.opModeIsActive() && !autoOpMode.isStopRequested() && !exit) {
            //TODO Uncomment after updating to 1.8 terminology
            /*if (isNotEqual(currentPose, driveTrain.getPoseEstimate())) {
                trajectoryCorrection = driveTrain.trajectorySequenceBuilder(driveTrain.getPoseEstimate())
                        .lineToLinearHeading(currentPose)
                        .build();
                driveTrain.followTrajectorySequence(trajectoryCorrection);
            }*/
        }
    }

    public void exit(){
        exit = true;
    }

    public boolean isNotEqual(Pose2d pose1, Pose2d pose2) {
        return (pose1.position.x - pose2.position.x > 0.5 ||
            pose1.position.y - pose2.position.y > 0.5 ||
                pose1.heading.log() - pose2.heading.log() > Math.toRadians(0.5));
    }
}
