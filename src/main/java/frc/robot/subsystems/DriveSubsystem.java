package frc.robot.subsystems;

import frc.robot.consts;
import frc.robot.utils.TunableNumber;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
// import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// TODO: Feedforward PID

public class DriveSubsystem extends SubsystemBase {
  private final TalonFX motorFrontRight = new TalonFX(consts.CANID.RCanIDci);
  private final TalonFX motorFrontLeft = new TalonFX(consts.CANID.LCanIDci);
  private final PIDController pid = new PIDController(consts.PosPID.posKPcd, consts.PosPID.posKIcd, consts.PosPID.posKDcd);

  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
  // private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

  // Tunable numbers for PID constants
  private final TunableNumber kP = new TunableNumber("DriveSubsystem/kP", consts.VelPID.velKPcd);
  private final TunableNumber kI = new TunableNumber("DriveSubsystem/kI", consts.VelPID.velKIcd);
  private final TunableNumber kD = new TunableNumber("DriveSubsystem/kD", consts.VelPID.velKDcd);

  /** Enum to track which PID mode is active */
  private enum DriveMode {
    VELOCITY, POSITION
  }

  private DriveMode currentMode = null;

  public DriveSubsystem() {
    motorFrontLeft.getConfigurator().apply(genConfig(true));
    motorFrontRight.getConfigurator().apply(genConfig(false));
  }

  public TalonFXConfiguration genConfig(boolean inverted) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    if (inverted) {
      config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    } else {
      config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    }

    Slot0Configs slot0 = config.Slot0;
    // Retrieve PID values from TunableNumber instances
    slot0.kP = kP.get();
    slot0.kI = kI.get();
    slot0.kD = kD.get();
    slot0.kV = consts.VelPID.velKVcd;

    return config;
  }

  /** Velocity PID control (RPM) */
  public void setVelocity(double leftRPM, double rightRPM) {
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftRPM));
    motorFrontRight.setControl(velocityRequest.withVelocity(rightRPM));

    currentMode = DriveMode.VELOCITY; // Update current mode
  }

  /** Position PID control (rotations) */
  public void setPosition(double leftRotations, double rightRotations) {
    // Get current positions
    double currentLeftRot = motorFrontLeft.getPosition().getValueAsDouble();
    double currentRightRot = motorFrontRight.getPosition().getValueAsDouble();

    // Calculate velocity command using PID based on position error
    double leftVelocity = pid.calculate(currentLeftRot, leftRotations);
    double rightVelocity = pid.calculate(currentRightRot, rightRotations);

    // Apply calculated velocities
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftVelocity));
    motorFrontRight.setControl(velocityRequest.withVelocity(rightVelocity));

    currentMode = DriveMode.POSITION; // Update current mode
  }

  @Override
  public void periodic() {
    // Log current control mode
    Logger.recordOutput("Drive/Mode", currentMode == null ? "NONE" : currentMode.name());

    // Update motor configurations periodically to reflect changes in PID values
    motorFrontLeft.getConfigurator().apply(genConfig(true));
    motorFrontRight.getConfigurator().apply(genConfig(false));

    // LEFT motor logs
    Logger.recordOutput("Drive/Left/Temp", motorFrontLeft.getDeviceTemp().getValue());
    Logger.recordOutput("Drive/Left/VelocityRPM", motorFrontLeft.getVelocity().getValue());
    Logger.recordOutput("Drive/Left/Position", motorFrontLeft.getPosition().getValue());
    Logger.recordOutput("Drive/Left/Voltage", motorFrontLeft.getMotorVoltage().getValue());
    Logger.recordOutput("Drive/Left/Current", motorFrontLeft.getStatorCurrent().getValue());
    Logger.recordOutput("Drive/Left/OutputPercent", motorFrontLeft.getDutyCycle().getValue());

    // RIGHT motor logs
    Logger.recordOutput("Drive/Right/Temp", motorFrontRight.getDeviceTemp().getValue());
    Logger.recordOutput("Drive/Right/VelocityRPM", motorFrontRight.getVelocity().getValue());
    Logger.recordOutput("Drive/Right/Position", motorFrontRight.getPosition().getValue());
    Logger.recordOutput("Drive/Right/Voltage", motorFrontRight.getMotorVoltage().getValue());
    Logger.recordOutput("Drive/Right/Current", motorFrontRight.getStatorCurrent().getValue());
    Logger.recordOutput("Drive/Right/OutputPercent", motorFrontRight.getDutyCycle().getValue());
  }
}