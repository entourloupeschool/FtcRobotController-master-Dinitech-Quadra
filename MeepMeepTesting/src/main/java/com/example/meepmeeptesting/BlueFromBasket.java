package com.example.meepmeeptesting;

import static com.example.meepmeeptesting.MeepMeepTesting.CLOSE_SHOOT_POSE;
import static com.example.meepmeeptesting.MeepMeepTesting.FIRST_ROW_ARTEFACTS_PREP_POSE;
import static com.example.meepmeeptesting.MeepMeepTesting.LENGTH_ARTEFACT_ROW;
import static com.example.meepmeeptesting.MeepMeepTesting.OBELISK_POSE;
import static com.example.meepmeeptesting.MeepMeepTesting.SECOND_ROW_ARTEFACTS_PREP_POSE;
import static com.example.meepmeeptesting.MeepMeepTesting.THIRD_ROW_ARTEFACTS_PREP_POSE;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class BlueFromBasket extends Action {

    public Action BlueFromBasket(RoadRunnerBotEntity myBot, Pose2d blueGoalBeginPose) {
        return myBot.getDrive().actionBuilder(blueGoalBeginPose)
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
                .build();
    }
}
