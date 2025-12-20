package org.firstinspires.ftc.teamcode.dinitech.opmodes.auto;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.shooter.MaxSpeedShooter;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.MoulinCalibrate;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.trieur.ReadyMotifAuto;
import org.firstinspires.ftc.teamcode.dinitech.commands.basecommands.vision.ContinuousUpdateAprilTagsDetections;
import org.firstinspires.ftc.teamcode.dinitech.opmodes.DinitechRobotBase;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.DinitechMecanumDrive;

@Autonomous(name = "GornetixAutoBlueGoal - Dinitech", group = "Auto")
public class GornetixAutoBlueGoal extends DinitechRobotBase {
        private TrieurSubsystem trieurSubsystem;
        private VisionSubsystem visionSubsystem;

        private ShooterSubsystem shooterSubsystem;
        private ChargeurSubsystem chargeurSubsystem;
        private DriveSubsystem driveSubsystem;

        /**
         * Initialize the teleop OpMode, gamepads, buttons, and default commands.
         */
        @Override
        public void initialize() {
                super.initialize();

                visionSubsystem = new VisionSubsystem(hardwareMap, telemetry);
                visionSubsystem.addAprilTagProcessor();
                visionSubsystem.buildVisionPortal();
                register(visionSubsystem);
                visionSubsystem.setDefaultCommand(new ContinuousUpdateAprilTagsDetections(visionSubsystem));

                driveSubsystem = new DriveSubsystem(new DinitechMecanumDrive(hardwareMap, BEGIN_POSE),
                                telemetry);
                register(driveSubsystem);

                trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetry);
                register(trieurSubsystem);

                // trieurSubsystem.setMoulinStateColor();

                chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetry);
                register(chargeurSubsystem);

                shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetry);
                register(shooterSubsystem);

                new MoulinCalibrate(trieurSubsystem).schedule();

                // LISTENER: Automatic trigger: when trieur becomes full, spin up shooter to max speed and
                // Moulin Motif Ready
                new Trigger(trieurSubsystem::getIsFull)
                        .whenActive(new ParallelCommandGroup(
                                new ReadyMotifAuto(trieurSubsystem, visionSubsystem),
                                new MaxSpeedShooter(shooterSubsystem)
                        ));

                // See the Obelisk


                // Set trieur moulinColor .setMoulinStoragePositionColor()
                autoSetArtefactColors();
                trieurSubsystem.setIsFull(true);

                // Go to shooting position


                // Shoot Revolution


                // For loop 3 :
                        // Go to next artefacts row

                        // Automatic pick up

        }

        /**
         * Main OpMode loop. Updates gamepad states.
         */
        @Override
        public void run() {
                super.run();
        }

        /**
         * auto set artefact colors
         */
        private void autoSetArtefactColors(){
                trieurSubsystem.setMoulinStoragePositionColor(1, TrieurSubsystem.ArtifactColor.PURPLE);
                trieurSubsystem.setMoulinStoragePositionColor(2, TrieurSubsystem.ArtifactColor.PURPLE);
                trieurSubsystem.setMoulinStoragePositionColor(3, TrieurSubsystem.ArtifactColor.GREEN);
        }
}
