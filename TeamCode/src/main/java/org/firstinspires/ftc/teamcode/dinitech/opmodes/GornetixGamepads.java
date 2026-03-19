package org.firstinspires.ftc.teamcode.dinitech.opmodes;


import org.firstinspires.ftc.teamcode.dinitech.commands.baseCommands.hub.DefaultHubsCommand;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ChargeurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.DrivePedroSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.GamepadSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.TrieurSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.VisionSubsystem;
import org.firstinspires.ftc.teamcode.dinitech.subsytems.devices.GamepadWrapper;

public class GornetixGamepads extends Gornetix {
    public GamepadSubsystem gamepadSubsystem;
    public GamepadWrapper m_Driver;
    public GamepadWrapper m_Operator;

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
    }

}
