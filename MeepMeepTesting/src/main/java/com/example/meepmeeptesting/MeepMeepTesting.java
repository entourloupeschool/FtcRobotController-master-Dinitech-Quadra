package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    static double botWidthCM = 34.5;
    static double botLengthCM = 38.5;
    static double cmToInch = 1.0 / 2.54;

    static double TILE_DIM = 24;
    static double LENGTH_ARTEFACT_ROW = TILE_DIM * 0.9;
    static double DISTANCE_BETWEEN_ARTEFACT_ROW = TILE_DIM * 1;

    static Pose2d BLUE_GOAL_BEGIN_POSE = new Pose2d(-2 * TILE_DIM, -2 * TILE_DIM, Math.PI / 4);
    static Pose2d OBELISK_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, Math.PI);
    static Pose2d CLOSE_SHOOT_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, 1.25 * Math.PI);

    static Pose2d FIRST_ROW_ARTEFACTS_PREP_POSE = new Pose2d(-TILE_DIM / 2, -1.1*TILE_DIM, Math.PI/2);
    static Pose2d SECOND_ROW_ARTEFACTS_PREP_POSE = new Pose2d(new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW, FIRST_ROW_ARTEFACTS_PREP_POSE.position.y), FIRST_ROW_ARTEFACTS_PREP_POSE.heading);
    static Pose2d THIRD_ROW_ARTEFACTS_PREP_POSE = new Pose2d(new Vector2d(SECOND_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW, SECOND_ROW_ARTEFACTS_PREP_POSE.position.y), SECOND_ROW_ARTEFACTS_PREP_POSE.heading);

    static  Pose2d FULL_BASE_POSE = new Pose2d(1.55 * TILE_DIM, -1.35 * TILE_DIM, Math.PI);

    public static void main(String[] args) {
            MeepMeep meepMeep = new MeepMeep(950);

            RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                            .setDimensions(botWidthCM * cmToInch, botLengthCM * cmToInch)
                            .build();

            myBot.runAction(new BlueFromBasket());

            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                            .setBackgroundAlpha(0.95f)
                            .addEntity(myBot)
                            .start();
    }


}