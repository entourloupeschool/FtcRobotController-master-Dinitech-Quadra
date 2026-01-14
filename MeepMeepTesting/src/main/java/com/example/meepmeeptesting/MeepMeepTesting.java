package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
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

    static Pose2d BLUE_END_GAME_POSE = new Pose2d(TILE_DIM / 2, -TILE_DIM / 2, Math.PI);


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

//            myBot.runAction(BlueFromBasket(myBot, BLUE_GOAL_BEGIN_POSE));
            myBot.runAction(RectangleCouleurs(myBot, new Pose2d(0, 0, Math.PI*1.2)));

            meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                            .setBackgroundAlpha(0.95f)
                            .addEntity(myBot)
                            .start();
    }

    public static Action RectangleCouleurs(RoadRunnerBotEntity myBot, Pose2d beginPose) {
        double x_rectangle = 1.55*TILE_DIM;
        double y_rectangle = 1.34*TILE_DIM;

        Pose2d RectangleRouge = new Pose2d(x_rectangle, -y_rectangle, 1.05*Math.PI);
        Pose2d RectangleBleu = new Pose2d(x_rectangle, y_rectangle, 1.25*Math.PI);
        Pose2d OpposeRectangleRouge = new Pose2d(-x_rectangle, -y_rectangle, 1.2*Math.PI);
        Pose2d OpposeRectangleBleu = new Pose2d(-x_rectangle, y_rectangle, 1.45*Math.PI);
        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(beginPose)
                .strafeToLinearHeading(RectangleRouge.position, RectangleRouge.heading)
                .strafeToLinearHeading(RectangleBleu.position, RectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleBleu.position, OpposeRectangleBleu.heading)
                .strafeToLinearHeading(OpposeRectangleRouge.position, OpposeRectangleRouge.heading)
                .strafeToLinearHeading(new Vector2d(0, 0), Math.PI);


        return builder.build();
    }

    public static Action Rosace(RoadRunnerBotEntity myBot, Pose2d beginPose) {
        double Rosace_length = 3 * botLengthCM * cmToInch;
        TrajectoryActionBuilder builder = myBot.getDrive().actionBuilder(beginPose);

        double tranchePI = 4;
        for (int i=1; i < tranchePI*2; i++){
            double angleOffset = i * Math.PI / tranchePI;

            //Calculate angles for the Left and Right petals
            double leftAngle = -Math.PI / 2 + angleOffset;
            double rightAngle = Math.PI / 2 + angleOffset;

            //Define the Poses for the tips of the petals
            Pose2d leftPetalTip = new Pose2d(
                    new Vector2d(
                            beginPose.position.x + Rosace_length * Math.cos(leftAngle),
                            beginPose.position.y + Rosace_length * Math.sin(leftAngle)
                    ),
                    beginPose.heading // Face the direction of the petal
            );
            Pose2d rightPetalTip = new Pose2d(
                    new Vector2d(
                            beginPose.position.x + Rosace_length * Math.cos(rightAngle),
                            beginPose.position.y + Rosace_length * Math.sin(rightAngle)
                    ),
                    beginPose.heading // Face the direction of the petal
            );

            builder = builder.setTangent(angleOffset)
                    .splineToLinearHeading(leftPetalTip, angleOffset)
                    .splineToLinearHeading(beginPose, angleOffset)
                    .splineToLinearHeading(rightPetalTip, angleOffset)
                    .splineToLinearHeading(beginPose, angleOffset);
        }
        return builder.build();
    }



    public static Action BlueFromBasket(RoadRunnerBotEntity myBot, Pose2d beginPose) {
        return myBot.getDrive().actionBuilder(beginPose)
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