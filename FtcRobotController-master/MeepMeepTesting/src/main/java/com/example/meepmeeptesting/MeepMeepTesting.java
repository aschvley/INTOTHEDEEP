package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;

import org.rowlandhall.meepmeep.MeepMeep;
import org.rowlandhall.meepmeep.roadrunner.DefaultBotBuilder;
import org.rowlandhall.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65.70405415534033, 65.70405415534033, Math.toRadians(258.37), Math.toRadians(258.37), 14.57)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(new Pose2d(26, -63.5, Math.toRadians(90)))
                        //go to hang preloaded specimen
                        .lineTo(new Vector2d(0, -30))
                        .lineToSplineHeading(new Pose2d(35, -36, Math.toRadians(90)))
                        //parks in ascent 1
                        .lineTo(new Vector2d(58, -60))
                        .build());


        meepMeep.setBackground(MeepMeep.Background.FIELD_INTOTHEDEEP_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}