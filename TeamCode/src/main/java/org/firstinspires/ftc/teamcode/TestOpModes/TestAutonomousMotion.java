package org.firstinspires.ftc.teamcode.TestOpModes;
import androidx.appcompat.graphics.drawable.DrawerArrowDrawable;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.TankDrive;
import org.firstinspires.ftc.teamcode.RRDrive.tuning.TuningOpModes;

@Autonomous(name = "TestAutonomousMotion", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp Thread")
@Disabled
public final class TestAutonomousMotion extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

        waitForStart();

        Actions.runBlocking(
                drive.actionBuilder(drive.pose)
                        .splineTo(new Vector2d(28, -3), Math.toDegrees(90)) //Math.PI / 2
                        //.splineTo(new Vector2d(60, 0), Math.PI)
                        .build());

        drive.updatePoseEstimate();

        telemetry.addData("x", drive.pose.position.x);
        telemetry.addData("y", drive.pose.position.y);
        telemetry.addData("heading (deg)", Math.toDegrees(drive.pose.heading.toDouble()));
        telemetry.update();

        sleep(5000);

    }
}