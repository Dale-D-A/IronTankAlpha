package frc.robot.subsystems;

import frc.robot.consts;

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
  private final PIDController pid = new PIDController(0, 0, 0);

  private final VelocityDutyCycle velocityRequest = new VelocityDutyCycle(0);
  // private final PositionDutyCycle positionRequest = new PositionDutyCycle(0);

  /** Enum to track which PID mode is active */
  private enum DriveMode {
    VELOCITY, POSITION
  }

  private DriveMode currentMode = null;

  public DriveSubsystem() {
    configureForVelocity(); // Default
  }

  /** Apply velocity PID settings if not already applied */
  private void configureForVelocity() {
    if (currentMode == DriveMode.VELOCITY) return;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = consts.VelPID.velKPcd;
    slot0.kI = consts.VelPID.velKIcd;
    slot0.kD = consts.VelPID.velKDcd;
    slot0.kV = consts.VelPID.velKVcd;

    motorFrontLeft.getConfigurator().apply(config);
    motorFrontRight.getConfigurator().apply(config);

    currentMode = DriveMode.VELOCITY;
  }

  /** Apply position PID settings if not already applied */
  private void configureForPosition() {
    if (currentMode == DriveMode.POSITION) return;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = consts.PosPID.posKPcd;
    slot0.kI = consts.PosPID.posKIcd;
    slot0.kD = consts.PosPID.posKDcd;

    motorFrontLeft.getConfigurator().apply(config);
    motorFrontRight.getConfigurator().apply(config);

    currentMode = DriveMode.POSITION;
  }

  /** Velocity PID control (RPM) */
  public void setVelocity(double leftRPM, double rightRPM) {
    configureForVelocity();
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftRPM));
    motorFrontRight.setControl(velocityRequest.withVelocity(-rightRPM));
  }

  /** Position PID control (rotations) */
  public void setPosition(double leftRotations, double rightRotations) {
    configureForPosition();

    // Get current positions
    double currentLeftRot = motorFrontLeft.getPosition().getValueAsDouble();
    double currentRightRot = motorFrontRight.getPosition().getValueAsDouble();

    // Calculate velocity command using PID based on position error
    double leftVelocity = pid.calculate(currentLeftRot, leftRotations);
    double rightVelocity = pid.calculate(currentRightRot, rightRotations);

    // Apply calculated velocities
    motorFrontLeft.setControl(velocityRequest.withVelocity(leftVelocity));
    motorFrontRight.setControl(velocityRequest.withVelocity(rightVelocity));
  }

  @Override
  public void periodic() {
    // Log current control mode
    Logger.recordOutput("Drive/Mode", currentMode == null ? "NONE" : currentMode.name());

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