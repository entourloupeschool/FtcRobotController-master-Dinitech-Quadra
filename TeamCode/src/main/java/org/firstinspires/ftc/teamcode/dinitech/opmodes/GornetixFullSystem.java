package org.firstinspires.ftc.teamcode.dinitech.opmodes;

import static org.firstinspires.ftc.teamcode.dinitech.other.Globals.BEGIN_POSE;

import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.DefaultHubsCommand;
import org.firstinspires.ftc.teamcode.dinitech.other.PoseStorage;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

import java.util.Objects;

public class GornetixFullSystem extends GornetixRobotBase {
    public GamepadSubsystem gamepadSubsystem;
    public GamepadWrapper m_Driver;
    public GamepadWrapper m_Operator;
    public TrieurSubsystem trieurSubsystem;
    public VisionSubsystem visionSubsystem;
    public ShooterSubsystem shooterSubsystem;
    public ChargeurSubsystem chargeurSubsystem;
    public DrivePedroSubsystem drivePedroSubsystem;

    /**
     * Initialize all hardware and subsystems.
     */
    @Override
    public void initialize() {
        super.initialize();

        gamepadSubsystem = new GamepadSubsystem(gamepad1, gamepad2, telemetryM);
        register(gamepadSubsystem);
        m_Driver = gamepadSubsystem.getDriver();
        m_Operator = gamepadSubsystem.getOperator();

        visionSubsystem = new VisionSubsystem(hardwareMap, telemetryM);
        register(visionSubsystem);

        drivePedroSubsystem = new DrivePedroSubsystem(hardwareMap, telemetryM);
        register(drivePedroSubsystem);

        trieurSubsystem = new TrieurSubsystem(hardwareMap, telemetryM);
        register(trieurSubsystem);

        chargeurSubsystem = new ChargeurSubsystem(hardwareMap, telemetryM);
        register(chargeurSubsystem);

        shooterSubsystem = new ShooterSubsystem(hardwareMap, telemetryM);
        register(shooterSubsystem);

        hubsSubsystem.setDefaultCommand(new DefaultHubsCommand(hubsSubsystem, trieurSubsystem, this::getOnBlueTeam));
    }

    /**
     * Main OpMode loop. Handles voltage monitoring and cache clearing.
     */
    @Override
    public void run() {
        super.run();
    }

}
