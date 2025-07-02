package frc.robot;

import frc.robot.utils.TunableNumber;

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
    public static final TunableNumber DRIVE_VELOCITY_KP = new TunableNumber("00",0);
    public static final double velKIcd = 0.0;
    public static final double velKDcd = 0.0;
    public static final double velKVcd = 0.12;
  }

  // Position PID
  public static final class PosPID {
    public static final double posKPcd = 1.5;
    public static final double posKIcd = 0.0;
    public static final double posKDcd = 0.05;
  }

  // Max values
  public static final class Maximums {
    // TODO: This is an example. Value is untested
    // This value controls the maximum of which the drive motors can run in
    // Real drive RPM = proportion * maxDriveRPMcd, proportion in [0, 1]
    public static final double maxDriveRPMcd = 1000.0;
  }
  
}
