package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.core.colorscheme.scheme.ColorSchemeRedDark;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;
public class MeepMeep2 {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(900)
                .setBackground(MeepMeep.Background.FIELD_FREIGHTFRENZY_OFFICIAL)
                .setTheme(new ColorSchemeRedDark())
                .setBackgroundAlpha(1f)
                ;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setConstraints(60,60,Math.toRadians(423),Math.toRadians(423), 15)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(10,64,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-5,42,Math.toRadians(70)))
                                //loop 1
                        .lineToLinearHeading(new Pose2d(10,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(42,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(10,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-5,42, Math.toRadians(70)))
                                //loop 2
                        .lineToLinearHeading(new Pose2d(10,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(42, 64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24, 64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(10, 64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-5,42, Math.toRadians(70)))
                                //loop 3
                        .lineToLinearHeading(new Pose2d(10,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24, 64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(42,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24,64, Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(10,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(-5, 42, Math.toRadians(70)))
                                //Parking in warehouse
                        .lineToLinearHeading(new Pose2d(10,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(24,64,Math.toRadians(0)))
                        .lineToLinearHeading(new Pose2d(42,64,Math.toRadians(0)))
                        .build()


                );
        meepMeep.addEntity(myBot)
                .start();



    }
}
