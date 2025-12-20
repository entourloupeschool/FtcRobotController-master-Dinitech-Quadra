package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Twist2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
        public static void main(String[] args) {
                MeepMeep meepMeep = new MeepMeep(950);

                double botWidthCM = 34.5;
                double botLengthCM = 38.5;
                double cmToInch = 1.0 / 2.54;

                double TILE_DIM = 24;
                double LENGTH_ARTEFACT_ROW = TILE_DIM * 0.9;
                double DISTANCE_BETWEEN_ARTEFACT_ROW = TILE_DIM * 1;

                Pose2d BLUE_GOAL_BEGIN_POSE = new Pose2d(-2 * TILE_DIM, -2 * TILE_DIM, Math.PI / 4);
                Pose2d OBELISK_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, Math.PI);
                Pose2d CLOSE_SHOOT_POSE = new Pose2d(-TILE_DIM / 2, -TILE_DIM / 2, 1.25 * Math.PI);

                Pose2d FIRST_ROW_ARTEFACTS_PREP_POSE = new Pose2d(-TILE_DIM / 2, -1.1*TILE_DIM, Math.PI/2);
                Pose2d SECOND_ROW_ARTEFACTS_PREP_POSE = new Pose2d(new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW, FIRST_ROW_ARTEFACTS_PREP_POSE.position.y), FIRST_ROW_ARTEFACTS_PREP_POSE.heading);
                Pose2d THIRD_ROW_ARTEFACTS_PREP_POSE = new Pose2d(new Vector2d(SECOND_ROW_ARTEFACTS_PREP_POSE.position.x + DISTANCE_BETWEEN_ARTEFACT_ROW, SECOND_ROW_ARTEFACTS_PREP_POSE.position.y), SECOND_ROW_ARTEFACTS_PREP_POSE.heading);

                Pose2d FULL_BASE_POSE = new Pose2d(1.55 * TILE_DIM, -1.35 * TILE_DIM, Math.PI);

                RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                                .setDimensions(botWidthCM * cmToInch, botLengthCM * cmToInch)
                                .build();

                myBot.runAction(myBot.getDrive().actionBuilder(BLUE_GOAL_BEGIN_POSE)
                        // See the obelisk
                        .strafeToLinearHeading(OBELISK_POSE.position, OBELISK_POSE.heading)

                        // Trieur busines

                        // Shoot close - just rotate since we're already at the right position
                        .turnTo(CLOSE_SHOOT_POSE.heading)

                        // go to next row of artefacts
                        .strafeToLinearHeading(FIRST_ROW_ARTEFACTS_PREP_POSE.position, FIRST_ROW_ARTEFACTS_PREP_POSE.heading)
                        .strafeToConstantHeading(new Vector2d(FIRST_ROW_ARTEFACTS_PREP_POSE.position.x, FIRST_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                        // Shoot close
                        .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)

                        // go to next row of artefacts
                        .strafeToLinearHeading(SECOND_ROW_ARTEFACTS_PREP_POSE.position, SECOND_ROW_ARTEFACTS_PREP_POSE.heading)
                        .strafeToConstantHeading(new Vector2d(SECOND_ROW_ARTEFACTS_PREP_POSE.position.x, SECOND_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                        // Shoot close
                        .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)

                        // go to next row of artefacts
                        .strafeToLinearHeading(THIRD_ROW_ARTEFACTS_PREP_POSE.position, THIRD_ROW_ARTEFACTS_PREP_POSE.heading)
                        .strafeToConstantHeading(new Vector2d(THIRD_ROW_ARTEFACTS_PREP_POSE.position.x, THIRD_ROW_ARTEFACTS_PREP_POSE.position.y - LENGTH_ARTEFACT_ROW))

                        // Shoot close
                        .strafeToLinearHeading(CLOSE_SHOOT_POSE.position, CLOSE_SHOOT_POSE.heading)
                        .build());

                meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                                .setBackgroundAlpha(0.95f)
                                .addEntity(myBot)
                                .start();
        }
}