package frc.robot;

public class consts {
    public static final class CanID {
        public static final int motorFrontLeftID = 0;
        public static final int motorFrontRightID = 1;
        public static final int motorTest = 2;
    }

    public static final class PID {
        public static final double KP = 5; // Proportional gain
        public static final double KI = 0.0; // Integral gain
        public static final double KD = 0.0; // Derivative gain
    }

    public static final double testTurns = 5; // Number of turns to test
}
