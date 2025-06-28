package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts;

public class DriveSubsystem extends SubsystemBase {
  // Motor controllers
  WPI_VictorSPX motorFrontRight = new WPI_VictorSPX(consts.CanID.motorFrontRight);
  WPI_VictorSPX motorFrontLeft = new WPI_VictorSPX(consts.CanID.motorFrontLeft);

  // PID constants
  private static final double kP = consts.PID.kP; // Proportional gain
  private static final double kI = consts.PID.kI; // Integral gain
  private static final double kD = consts.PID.kD; // Derivative gain

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Set motor inversion
    motorFrontLeft.setInverted(false);
    motorFrontRight.setInverted(false);

    // Set brake mode
    motorFrontLeft.setNeutralMode(NeutralMode.Brake);
    motorFrontRight.setNeutralMode(NeutralMode.Brake);

    // Configure encoders
    motorFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motorFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    // Configure PID for velocity control
    configPID(kP, kI, kD);

    // Reset encoders to zero
    resetEncoders();
  }

  public void configPID(double kP, double kI, double kD) {
    motorFrontLeft.config_kP(0, kP);
    motorFrontLeft.config_kI(0, kI);
    motorFrontLeft.config_kD(0, kD);

    motorFrontRight.config_kP(0, kP);
    motorFrontRight.config_kI(0, kI);
    motorFrontRight.config_kD(0, kD);
  }

  /** Set the target velocity for the motors in meters per second. */
  public void setVelocity(double leftVelocityMetersPerSecond, double rightVelocityMetersPerSecond) {
    double leftVelocityTicksPer100ms = leftVelocityMetersPerSecond * consts.Encoder.ticksPerMeter / 10.0;
    double rightVelocityTicksPer100ms = rightVelocityMetersPerSecond * consts.Encoder.ticksPerMeter / 10.0;

    motorFrontLeft.set(ControlMode.Velocity, leftVelocityTicksPer100ms);
    motorFrontRight.set(ControlMode.Velocity, rightVelocityTicksPer100ms);
  }

  /** Get the velocity of the left motor in meters per second. */
  public double getLeftVelocityMetersPerSecond() {
    return (motorFrontLeft.getSelectedSensorVelocity() * 10.0) / consts.Encoder.ticksPerMeter;
  }

  /** Get the velocity of the right motor in meters per second. */
  public double getRightVelocityMetersPerSecond() {
    return (motorFrontRight.getSelectedSensorVelocity() * 10.0) / consts.Encoder.ticksPerMeter;
  }

  /** Reset the encoders to zero. */
  public void resetEncoders() {
    motorFrontLeft.setSelectedSensorPosition(0);
    motorFrontRight.setSelectedSensorPosition(0);
  }

  /** Run motor according to joystick input values. This is DEPRECATED. */
  public void setSpeeds(double leftSpeeds, double rightSpeeds) {
    motorFrontLeft.set(ControlMode.PercentOutput, leftSpeeds);
    motorFrontRight.set(ControlMode.PercentOutput, -rightSpeeds);
  }

  /** Run arcade drive based on velocity control. */
  public void setArcadeSpeed(double forwardSpeedMetersPerSecond, double turningSpeedMetersPerSecond) {
    double leftSpeed = forwardSpeedMetersPerSecond + turningSpeedMetersPerSecond;
    double rightSpeed = forwardSpeedMetersPerSecond - turningSpeedMetersPerSecond;

    setVelocity(leftSpeed, rightSpeed);
  }

  @Override
  public void periodic() {
    // Log encoder data
    Logger.recordOutput("Left Distance (m)", getLeftDistanceMeters());
    Logger.recordOutput("Right Distance (m)", getRightDistanceMeters());
    Logger.recordOutput("Left Velocity (m/s)", getLeftVelocityMetersPerSecond());
    Logger.recordOutput("Right Velocity (m/s)", getRightVelocityMetersPerSecond());
  }

  /** Get the distance traveled by the left motor in meters. */
  public double getLeftDistanceMeters() {
    return (motorFrontLeft.getSelectedSensorPosition() / consts.Encoder.ticksPerRevolution) * (Math.PI * consts.Encoder.wheelDiameterMeters);
  }

  /** Get the distance traveled by the right motor in meters. */
  public double getRightDistanceMeters() {
    return (motorFrontRight.getSelectedSensorPosition() / consts.Encoder.ticksPerRevolution) * (Math.PI * consts.Encoder.wheelDiameterMeters);
  }
}