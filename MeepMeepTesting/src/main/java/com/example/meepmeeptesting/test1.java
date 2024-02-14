package com.example.meepmeeptesting;


import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class test1 {

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-35, -60, Math.PI / 2))
                .splineToLinearHeading(new Pose2d(-35, -50, 2.094395), 0)

                .waitSeconds(2)
                //.splineTo(new Vector2d(-10, -10), 2 * Math.PI)
                .splineToLinearHeading(new Pose2d(-10, -10, 2 * Math.PI), 0)

//                .setTangent(0.5)
//                .splineTo(new Vector2d(40, -35.5), 0)

//                .splineToLinearHeading(new Pose2d(50, -35.5, 1 * Math.PI), 0)
                .waitSeconds(2)
//                .setTangent(Math.PI)
//                .splineToConstantHeading(new Vector2d(-10, -10), Math.PI / 2)
//                .strafeTo(new Vector2d(-50, -10))
//                .splineToLinearHeading(new Pose2d(20, -10, 0), 0)
                .strafeTo(new Vector2d(20, -10))
                .splineToLinearHeading(new Pose2d(50, -35.5, 2 * Math.PI), -0.1)

//                .splineToLinearHeading(new Pose2d(50, -35.5, 2 * Math.PI), 3)
//                .strafeTo(new Vector2d(40, -35.5))
//                                .turn(Math.toRadians(180))
//                            .lineToY(-61.37)
//                            .setTangent(1)


                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}