package frc.robot.subsystems;
import frc.robot.consts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX motorArm = new TalonFX(consts.CANID.armCanIDci);
    public double armAngle; // Current angle of the arm in degrees
    private double targetPosition = 0;
    public ArmSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.withSlot0(new Slot0Configs().withKP(1).withKI(0).withKD(0));
        motorArm.getConfigurator().apply(config);
    }

    /**
     * Resets the current position of the arm to 0° (sets encoder to zero).
     */
    public void resetArmPositionToZero() {
        motorArm.setPosition(0);
        Logger.recordOutput("Arm/EncoderZeroed", true);
    }

    /**
     * Sets the arm position to a desired angle.
     * @param targetAngle The desired angle in degrees.
     */
    public void setArmPosition(double targetAngle) {
        targetPosition = targetAngle;
        motorArm.setControl(new VoltageOut(1));
    }

    @Override
    public void periodic() {
        // Log the current arm position every cycle
        Logger.recordOutput("Arm/Position", motorArm.getPosition().getValue());
        Logger.recordOutput("Arm/TargetAngle", targetPosition);
    }
}
