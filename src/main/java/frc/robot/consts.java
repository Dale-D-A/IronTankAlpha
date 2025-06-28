package frc.robot;

public class consts {
    public static final class CanID {
        public static final int motorFrontLeft = 0; // Front left motor controller ID
        public static final int motorFrontRight = 1; // Front right motor controller ID
    }

    public static final class PID {
        public static final double kP = 0.1; // Proportional gain
        public static final double kI = 0.0; // Integral gain
        public static final double kD = 0.0; // Derivative gain
    }

    public static final class Encoder {
        public static final double ticksPerRevolution = 4096; // Encoder ticks per revolution
        public static final double wheelDiameterMeters = 0.15; // Wheel diameter in meters
        public static final double ticksPerMeter = ticksPerRevolution / (Math.PI * wheelDiameterMeters); // Ticks per meter
    }

    public static final class Drive {
        public static final double maxRPM = 5000; // TODO: Update accordingly
    }
}
