package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.consts;

public class DriveSubsystem extends SubsystemBase {
  // Motor controllers
  private final WPI_VictorSPX motorFrontRight = new WPI_VictorSPX(consts.CanID.motorFrontRight);
  private final WPI_VictorSPX motorFrontLeft = new WPI_VictorSPX(consts.CanID.motorFrontLeft);

  // PID constants
  private static final double kP = consts.PID.kP;
  private static final double kI = consts.PID.kI;
  private static final double kD = consts.PID.kD;

  // Logging
  @AutoLog
  public class DriveSubsystemLog {
    public double leftRPM = getLeftRPM();
    public double rightRPM = getRightRPM();
    public double leftDistanceTicks = motorFrontLeft.getSelectedSensorPosition();
    public double rightDistanceTicks = motorFrontRight.getSelectedSensorPosition();
  }

  public DriveSubsystem() {
    motorFrontLeft.setInverted(false);
    motorFrontRight.setInverted(false);

    motorFrontLeft.setNeutralMode(NeutralMode.Brake);
    motorFrontRight.setNeutralMode(NeutralMode.Brake);

    motorFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
    motorFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);

    configPID(kP, kI, kD);
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

  /** Set motor speeds using RPM. */
  public void setRPM(double leftRPM, double rightRPM) {
    double ticksPer100msLeft = rpmToTicksPer100ms(leftRPM);
    double ticksPer100msRight = rpmToTicksPer100ms(rightRPM);

    motorFrontLeft.set(ControlMode.Velocity, ticksPer100msLeft);
    motorFrontRight.set(ControlMode.Velocity, ticksPer100msRight);
  }

  /** Convert RPM to encoder ticks per 100ms. */
  private double rpmToTicksPer100ms(double rpm) {
    return (rpm * consts.Encoder.ticksPerRevolution) / 600.0;
  }

  /** Convert encoder ticks per 100ms to RPM. */
  private double ticksPer100msToRPM(double ticksPer100ms) {
    return (ticksPer100ms * 600.0) / consts.Encoder.ticksPerRevolution;
  }

  public double getLeftRPM() {
    return ticksPer100msToRPM(motorFrontLeft.getSelectedSensorVelocity());
  }

  public double getRightRPM() {
    return ticksPer100msToRPM(motorFrontRight.getSelectedSensorVelocity());
  }

  public void resetEncoders() {
    motorFrontLeft.setSelectedSensorPosition(0);
    motorFrontRight.setSelectedSensorPosition(0);
  }

  /** DEPRECATED: Open-loop % output control. */
  @Deprecated
  public void setSpeeds(double leftPercent, double rightPercent) {
    motorFrontLeft.set(ControlMode.PercentOutput, leftPercent);
    motorFrontRight.set(ControlMode.PercentOutput, -rightPercent);
  }

  /** Arcade-style control using RPM setpoints. */
  public void setArcadeRPM(double forwardRPM, double turnRPM) {
    double leftRPM = forwardRPM + turnRPM;
    double rightRPM = forwardRPM - turnRPM;

    setRPM(leftRPM, rightRPM);
  }

  @Override
  public void periodic() {
    // Logger.recordOutput("Left RPM", getLeftRPM());
    // Logger.recordOutput("Right RPM", getRightRPM());
    // Logger.recordOutput("Left Distance (ticks)", motorFrontLeft.getSelectedSensorPosition());
    // Logger.recordOutput("Right Distance (ticks)", motorFrontRight.getSelectedSensorPosition());
  }

  /** Distance in ticks (can be converted to revolutions or meters externally if needed). */
  public double getLeftDistanceTicks() {
    return motorFrontLeft.getSelectedSensorPosition();
  }

  public double getRightDistanceTicks() {
    return motorFrontRight.getSelectedSensorPosition();
  }
}
