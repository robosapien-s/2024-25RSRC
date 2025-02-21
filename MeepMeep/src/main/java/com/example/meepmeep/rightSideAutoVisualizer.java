package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class rightSideAutoVisualizer {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                .setDimensions(16,16.5)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(-40,-62.5,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-60, -55,Math.toRadians(72)))
                        .lineToConstantHeading(new Vector2d(-57.25,-54))
                        .lineToLinearHeading(new Pose2d(-59.75, -56,Math.toRadians(93.5)))
                        .lineToLinearHeading(new Pose2d(-59.75, -55,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-59.75, -56,Math.toRadians(93.5)))//second
                        .lineToLinearHeading(new Pose2d(-52, -45,Math.toRadians(132)))
                        .lineToLinearHeading(new Pose2d(-61.5, -57,Math.toRadians(90)))
                        .lineToLinearHeading(new Pose2d(-40, -10,0))
                        .lineToConstantHeading(new Vector2d(-23,-10))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}