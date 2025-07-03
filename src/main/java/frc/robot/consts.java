package frc.robot;

import frc.robot.utils.TunableNumber;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/* const naming standards
 * nameName + c (const) + type
 */

public final class consts {
  // CAN IDs
  public static final class CANID {
    public static final int LCanIDci = 0;
    public static final int RCanIDci = 1;
    public static final int armCanIDci = 2;
  }
  public static final boolean TUNING = true;

  // Velocity PID
  public static final class VelPID {
    public static final TunableNumber DRIVE_VELOCITY_KP = new TunableNumber("Drive Velocity kP", 0.0);
    public static final TunableNumber DRIVE_VELOCITY_KI = new TunableNumber("Drive Velocity kI", 0.0);
    public static final TunableNumber DRIVE_VELOCITY_KD = new TunableNumber("Drive Velocity kD", 0.0);
    public static final TunableNumber DRIVE_VELOCITY_KV = new TunableNumber("Drive Velocity kV", 0.12);
  }

  // Max values
  public static final class Maximums {
    public static final double maxDriveRPMcd = 5000.0; // Example value
  }

  // Arm target angle
  public static final TunableNumber ARM_TARGET_ANGLE = new TunableNumber("Arm Target Angle", 30.0);
}
