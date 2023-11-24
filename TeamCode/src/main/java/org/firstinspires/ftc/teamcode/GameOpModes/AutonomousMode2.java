/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.GameOpModes;

import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.MILLISECONDS;
import static com.qualcomm.robotcore.util.ElapsedTime.Resolution.SECONDS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Controllers.GamepadController;
import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.SubSystems.Climber;
import org.firstinspires.ftc.teamcode.SubSystems.DriveTrain;
import org.firstinspires.ftc.teamcode.SubSystems.Intake;
import org.firstinspires.ftc.teamcode.SubSystems.Launcher;
import org.firstinspires.ftc.teamcode.SubSystems.Lights;
import org.firstinspires.ftc.teamcode.SubSystems.Magazine;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeArm;
import org.firstinspires.ftc.teamcode.SubSystems.OuttakeSlides;
import org.firstinspires.ftc.teamcode.SubSystems.VisionTfod;

/**
 * Hazmat Autonomous
 */
@Autonomous(name = "HazmatAutonomous Mode 2", group = "00-Autonomous", preselectTeleOp = "Hazmat TeleOp")
public class AutonomousMode2 extends LinearOpMode {

    //Static Class for knowing system state
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        //Game Play Button  is pressed
        if (opModeIsActive() && !isStopRequested()) {
            runAutonoumousMode();
        }
    }   // end runOpMode()

    public void runAutonoumousMode() {

        Pose2d center = new Pose2d(0, 0, Math.toRadians(0));
        Pose2d front = new Pose2d(10,0,Math.toRadians(0));
        Pose2d back = new Pose2d(-10,0,Math.toRadians(0));
        Pose2d middleLeft = new Pose2d(1,1,Math.toRadians(90));
        Pose2d left = new Pose2d(0,-10,Math.toRadians(90));
        Pose2d right = new Pose2d(0,10,Math.toRadians(90));

        MecanumDrive drive = new MecanumDrive(hardwareMap, center);

        Actions.runBlocking(
                drive.actionBuilder(center)
                        .strafeToLinearHeading(front.position, front.heading)
                        .strafeToLinearHeading(back.position, back.heading)
                        .strafeToLinearHeading(center.position, center.heading)
                        .strafeToLinearHeading(middleLeft.position, middleLeft.heading)
                        .strafeToLinearHeading(left.position, left.heading)
                        .strafeToLinearHeading(right.position, right.heading)
                        .strafeToLinearHeading(center.position, center.heading)
                        .build());
    }


    //method to wait safely with stop button working if needed. Use this instead of sleep
    public void safeWaitSeconds(double time) {
        ElapsedTime timer = new ElapsedTime(SECONDS);
        timer.reset();
        while (!isStopRequested() && timer.time() < time) {
        }
    }

    /**
     * Method to add debug messages. Update as telemetry.addData.
     * Use public attributes or methods if needs to be called here.
     */
    public void printDebugMessages(){
        telemetry.setAutoClear(true);
        telemetry.addData("DEBUG_LEVEL is : ", GameField.debugLevel);
        telemetry.addData("Robot ready to start","");

        if (GameField.debugLevel != GameField.DEBUG_LEVEL.NONE) {
            telemetry.addLine("Running Hazmat Autonomous Mode");
            //telemetry.addData("GameField.poseSetInAutonomous : ", GameField.poseSetInAutonomous);
            //telemetry.addData("GameField.currentPose : ", GameField.currentPose);
            //telemetry.addData("startPose : ", startPose);
        }
        telemetry.update();
    }


}   // end class
