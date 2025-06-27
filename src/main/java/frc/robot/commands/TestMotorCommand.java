package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.TestSubsystem;

import java.util.function.DoubleSupplier;

public class TestMotorCommand extends Command {
    private final TestSubsystem m_testSubsystem;
    private final DoubleSupplier positionSupplier;

    public TestMotorCommand(TestSubsystem testSubsystem, DoubleSupplier positionSupplier) {
        this.m_testSubsystem = testSubsystem;
        this.positionSupplier = positionSupplier;
        addRequirements(testSubsystem);
    }

    @Override
    public void execute() {
        m_testSubsystem.setMotor(positionSupplier.getAsDouble());
    }

    @Override
    public void end(boolean interrupted) {
        m_testSubsystem.stopMotor();
    }
}
