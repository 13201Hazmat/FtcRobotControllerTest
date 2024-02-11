package org.firstinspires.ftc.teamcode.SubSystems;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;


public class DriveTrain extends MecanumDrive{
    Telemetry telemetry;

    public DriveTrain(HardwareMap hardwareMap, Pose2d pose, Telemetry telemetry) {
        super(hardwareMap, pose);
        this.telemetry = telemetry;
    }

    public enum DriveType {
        ROBOT_CENTRIC,
        FIELD_CENTRIC,
    }
    public DriveType driveType = DriveType.ROBOT_CENTRIC;

    public Vector2d gamepadInput = new Vector2d(0,0);
    public double gamepadInputTurn = 0;

    public void driveNormal(){
        setDrivePowers(new PoseVelocity2d(
                new Vector2d(
                    gamepadInput.x,
                    gamepadInput.y),
                gamepadInputTurn
        ));

        updatePoseEstimate();
    }

    public Vector2d rotateFieldCentric(double x, double y, double angle){
        double newX, newY;
        newX = x * Math.cos(angle) - y * Math.sin(angle);
        newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }

    public String toStringVector2d(Vector2d vector){
        return String.format("(%.3f, %.3f)", vector.x, vector.y);
    }

    public String toStringPose2d(Pose2d pose){
        return String.format("(%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, Math.toDegrees(pose.heading.log()));
    }

    public void printDebugMessages(){
       //****** Drive debug ******
       telemetry.addData("Drive Type : ", driveType);
       telemetry.addData("PoseEstimateString :", toStringPose2d(pose));
       telemetry.addData("Parallel Left /LF Encoder", leftFront.getCurrentPosition());
       telemetry.addData("Parallel Right / LB Encoder", leftBack.getCurrentPosition());
       telemetry.addData("Perpendicular / RF Encoder", rightFront.getCurrentPosition());
       telemetry.addData("RB Encoder", rightBack.getCurrentPosition());
       telemetry.addData("LF motor is busy", leftFront.isBusy());
       telemetry.addData("LF motor is over current", leftFront.isOverCurrent());
       telemetry.addData("LB motor is busy", leftBack.isBusy());
       telemetry.addData("LB motor is over current", leftBack.isOverCurrent());
       telemetry.addData("RF motor is busy", rightFront.isBusy());
       telemetry.addData("RF motor is over current", rightFront.isOverCurrent());
       telemetry.addData("RB motor is busy", rightBack.isBusy());
       telemetry.addData("RB motor is over current", rightBack.isOverCurrent());

       telemetry.addLine("=============");
    }

}
