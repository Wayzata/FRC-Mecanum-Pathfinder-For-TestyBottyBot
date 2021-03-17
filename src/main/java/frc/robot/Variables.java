package frc.robot;

public class Variables {

    /**
     * Autonomous Constants are declared here
     */
    public class Auto{
        // Motor Ports
        public static final int frontLeft = 1;
        public static final int frontRight = 4;
        public static final int backLeft = 2;
        public static final int backRight = 3;

        // Measurements in INCHES
        public static final double kWheelRadiusInches = 4;
        public static final int kEncoderTicksPerRev = 1440;

        // Wheel positions in FEET
        public static final double kFrontRight_y = -0.96;
        public static final double kFrontRight_x = 0.83;
        public static final double kFrontLeft_y = 0.96;
        public static final double kFrontLeft_x = 0.83;
        public static final double kBackLeft_y = -0.96;
        public static final double kBackLeft_x = -0.83;
        public static final double kBackRight_y = -0.96;
        public static final double kBackRight_x = -0.83;

        // Speed Constraints
        public static final double kMaxSpeedFeetPerSecond = 10.0;
        public static final double kMaxAccelerationFeetPerSecond = 15.0;

        // PID Constants
        public static final double fl_kP = 1.12;
        public static final double fl_kI = 0;
        public static final double fl_kD = 0;

        public static final double bl_kP = 1.12;
        public static final double bl_kI = 0;
        public static final double bl_kD = 0;

        public static final double fr_kP = 1.12;
        public static final double fr_kI = 0;
        public static final double fr_kD = 0;

        public static final double br_kP = 1.12;
        public static final double br_kI = 0;
        public static final double br_kD = 0;
    }

}