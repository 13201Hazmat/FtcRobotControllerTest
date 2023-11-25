package org.firstinspires.ftc.teamcode.RRDrive.tuning;

import static org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive.PARAMS;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RRDrive.MecanumDrive;
import org.firstinspires.ftc.teamcode.RRDrive.TankDrive;
import org.firstinspires.ftc.teamcode.RRDrive.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.RRDrive.TwoDeadWheelLocalizer;
@Disabled
public final class ManualFeedbackTuner extends LinearOpMode {
    public static double DISTANCE = 64;

    enum Mode {
        DRIVER_MODE,
        TUNING_MODE
    }

    private Mode mode = Mode.TUNING_MODE;

    @Override
    public void runOpMode() throws InterruptedException {
        if (TuningOpModes.DRIVE_CLASS.equals(MecanumDrive.class)) {
            MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new AssertionError("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new AssertionError("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {

                telemetry.addData("mode", mode);

                switch (mode) {
                    case TUNING_MODE:
                        if (gamepad1.y) {
                            mode = Mode.DRIVER_MODE;
                        }


                        Actions.runBlocking(
                            drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());

                        PARAMS.axialVelGain = generateSMARTDerivativeTerm(PARAMS.axialGain, false);
                        PARAMS.lateralVelGain = generateSMARTDerivativeTerm(PARAMS.lateralGain, false);
                        PARAMS.headingVelGain = generateSMARTDerivativeTerm(PARAMS.headingGain, true);

                        telemetry.addLine("generateSMARTDerivativeTerm");
                        telemetry.addData("axialVelGain : ", generateSMARTDerivativeTerm(PARAMS.axialGain, false));
                        telemetry.addData("lateralVelGain : ", generateSMARTDerivativeTerm(PARAMS.lateralGain, false));
                        telemetry.addData("headingVelGain : ", generateSMARTDerivativeTerm(PARAMS.headingGain, true));

                        break;
                    case DRIVER_MODE:
                        if (gamepad1.b) {
                            mode = Mode.TUNING_MODE;
                        }
                        drive.setDrivePowers(new PoseVelocity2d(
                               new Vector2d(
                                        -gamepad1.left_stick_y,
                                        -gamepad1.left_stick_x
                                ),
                                -gamepad1.right_stick_x
                        ));

                        drive.updatePoseEstimate();

                        break;
                }
            }
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, new Pose2d(0, 0, 0));

            if (drive.localizer instanceof TwoDeadWheelLocalizer) {
                if (TwoDeadWheelLocalizer.PARAMS.perpXTicks == 0 && TwoDeadWheelLocalizer.PARAMS.parYTicks == 0) {
                    throw new AssertionError("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            } else if (drive.localizer instanceof ThreeDeadWheelLocalizer) {
                if (ThreeDeadWheelLocalizer.PARAMS.perpXTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par0YTicks == 0 && ThreeDeadWheelLocalizer.PARAMS.par1YTicks == 1) {
                    throw new AssertionError("Odometry wheel locations not set! Run AngularRampLogger to tune them.");
                }
            }
            waitForStart();

            while (opModeIsActive()) {
                Actions.runBlocking(
                        drive.actionBuilder(new Pose2d(0, 0, 0))
                                .lineToX(DISTANCE)
                                .lineToX(0)
                                .build());
            }
        } else {
            throw new AssertionError();
        }
    }

    //Hazmat added Generate damp function
    public static double generateSMARTDerivativeTerm(double kP, boolean rotation) {

        double kV = PARAMS.kV;
        double kA = PARAMS.kA;


        if (rotation) {
            kV /= PARAMS.trackWidthTicks * PARAMS.inPerTick;
            kA /= PARAMS.trackWidthTicks * PARAMS.inPerTick;
        }

        // anything below this will result in a non minimum phase system.
        // non-minimum phase means the system will hesitate, similar to that of a turning bicycle or a pitching aircraft.
        // while it would technically be faster, I have not yet proved it's stability so I will leave it out for now.
        double criticalKp = (kV * kV) / (4 * kA);
        if (criticalKp >= kP) {
            return 0;
        }
        // the critically damped PID derivative gain.
        return 2 * Math.sqrt(kA * kP) - kV;
    }


}
