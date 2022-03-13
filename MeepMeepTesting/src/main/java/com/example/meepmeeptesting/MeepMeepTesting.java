package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        DefaultBotBuilder defaultBotBuilder = new DefaultBotBuilder(meepMeep);
        defaultBotBuilder.setConstraints(50, 50, Math.toRadians(423), Math.toRadians(423), 15);// Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
        RoadRunnerBotEntity myBot = defaultBotBuilder
                .followTrajectorySequence(drive -> {
                    return drive.trajectorySequenceBuilder(new Pose2d(-72, 47, Math.toRadians(90))/*warehousePickElementPose[0]*/)
                            .lineToLinearHeading(new Pose2d(-72, 3, Math.toRadians(90))/*warehouseAllianceShippingPathPose[0]*/)
                            .addTemporalMarker(0.5, () -> {
                                //magazine.moveMagazineToTransport();
                                //intake.startIntakeMotorOutward();
                                //elevator.moveElevatorLevel3Position();
                            })
                            .lineToLinearHeading(new Pose2d(-32, -3, Math.toRadians(135)) /*allianceShippingHubDropElementPose*/)
                            .addTemporalMarker(() -> {
                                //magazine.moveMagazineToDrop();
                            })
                            .waitSeconds(0.8) // loopWait(800);
                            .lineToLinearHeading(new Pose2d(-73, 3, Math.toRadians(90))/*warehouseAllianceShippingPathPose[1]*/)
                            .addTemporalMarker(0.5, () -> {
                         /*magazine.moveMagazineToCollect();
                         elevator.moveElevatorLevel0Position();
                         intake.startIntakeMotorInward();
                          */
                            })
                            .lineToLinearHeading(new Pose2d(-73, 48, Math.toRadians(90)) /*warehousePickElementPose[1]*/)
                            .waitSeconds(0.8) // Instead of Sense;
                            //Second Loop
                            .lineToLinearHeading(new Pose2d(-73, 3, Math.toRadians(90)) /*warehouseAllianceShippingPathPose[1]*/)
                            .addTemporalMarker(0.5, () -> {
                         /* magazine.moveMagazineToTransport();
                         intake.startIntakeMotorOutward();
                         elevator.moveElevatorLevel3Position();
                          */
                            })
                            .lineToLinearHeading(new Pose2d(-32, -3, Math.toRadians(135)) /*allianceShippingHubDropElementPose */)
                            .addTemporalMarker(() -> {
                                //magazine.moveMagazineToDrop();
                            })
                            .waitSeconds(0.8) // loopWait(800);
                            .lineToLinearHeading(new Pose2d(-74, 3, Math.toRadians(90)) /*warehouseAllianceShippingPathPose[2] */)
                            .addTemporalMarker(0.5, () -> {
                         /*magazine.moveMagazineToCollect();
                         elevator.moveElevatorLevel0Position();
                         intake.startIntakeMotorInward();
                          */
                            })
                            .lineToLinearHeading(new Pose2d(-74, 52, Math.toRadians(90))/*warehousePickElementPose[2] */)
                            //ThirdLoop
                            .lineToLinearHeading(new Pose2d(-74, 3, Math.toRadians(90)) /*warehouseAllianceShippingPathPose[2] */)
                            .addTemporalMarker(0.5, () -> {
                         /*magazine.moveMagazineToTransport();
                         intake.startIntakeMotorOutward();
                         elevator.moveElevatorLevel3Position();

                          */
                            })
                            .lineToLinearHeading(new Pose2d(-32, -3, Math.toRadians(135)) /*allianceShippingHubDropElementPose*/)
                            .addTemporalMarker(() -> {
                                //magazine.moveMagazineToDrop();
                            })
                            .waitSeconds(0.8) // loopWait(800);
                            //Go to Park
                            .lineToLinearHeading(new Pose2d(-74, 3, Math.toRadians(90))/*warehouseAllianceShippingPathPose[2]*/)
                            .addTemporalMarker(0.5, () -> {
                         /*magazine.moveMagazineToCollect();
                         elevator.moveElevatorLevel0Position();
                          */
                                //intake.startIntakeMotorInward(); Dont pick
                            })
                            .lineToLinearHeading(new Pose2d(-74, 52, Math.toRadians(90)) /*warehousePickElementPose[2]*/)
                            .build();

        /*trajWarehouseAllianceShippingToParkBarrier= driveTrain.trajectorySequenceBuilder(alShippingHubPose)
                .addTemporalMarker(0,()->{moveElevatorToLevel(1);})
                .lineToLinearHeading(whThroughBarrierParkingPose[0])
                .lineToLinearHeading(whThroughBarrierParkingPose[1])
                .build();
                );
                );

         */
                        });


        meepMeep.setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_ADI_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
                }
    }