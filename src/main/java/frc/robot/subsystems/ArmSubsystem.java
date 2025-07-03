package frc.robot.subsystems;
import frc.robot.consts;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import static frc.robot.consts.ARM_KP;
import static frc.robot.consts.ARM_KI;
import static frc.robot.consts.ARM_KD;

public class ArmSubsystem extends SubsystemBase {
    private final TalonFX motorArm = new TalonFX(consts.CANID.armCanIDci);
    public double armAngle; // Current angle of the arm in degrees
    private double targetPosition = 0;
    private double lastKP = ARM_KP.get();
    private double lastKI = ARM_KI.get();
    private double lastKD = ARM_KD.get();
    public ArmSubsystem() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.withSlot0(new Slot0Configs().withKP(ARM_KP.get()).withKI(ARM_KI.get()).withKD(ARM_KD.get()));
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
        motorArm.setControl(new PositionDutyCycle(targetAngle));
    }

    @Override
    public void periodic() {
        // Update PID if changed
        if (ARM_KP.get() != lastKP || ARM_KI.get() != lastKI || ARM_KD.get() != lastKD) {
            Slot0Configs slot0 = new Slot0Configs()
                .withKP(ARM_KP.get())
                .withKI(ARM_KI.get())
                .withKD(ARM_KD.get());
            TalonFXConfiguration config = new TalonFXConfiguration();
            config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            config.withSlot0(slot0);
            motorArm.getConfigurator().apply(config);
            lastKP = ARM_KP.get();
            lastKI = ARM_KI.get();
            lastKD = ARM_KD.get();
        }
        // Log the current arm position every cycle
        Logger.recordOutput("Arm/Position", motorArm.getPosition().getValue());
        Logger.recordOutput("Arm/TargetAngle", targetPosition);
    }
}
