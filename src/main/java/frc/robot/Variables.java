package frc.robot;

public class Variables {

    /**
     * Autonomous Constants are declared here
     */
    public class Auto{
        // Motor Ports
        public static final int frontLeft = 3;
        public static final int frontRight = 0;
        public static final int backLeft = 2;
        public static final int backRight = 1;

        // Measurements in INCHES
        public static final double kWheelRadiusInches = 2.0;
        public static final int kEncoderTicksPerRev = 2048;

        // Wheel positions in FEET
        public static final double kFrontRight_y = -0.97916;
        public static final double kFrontRight_x = 0.875;
        public static final double kFrontLeft_y = 0.97916;
        public static final double kFrontLeft_x = 0.875;
        public static final double kBackLeft_y = -0.97916;
        public static final double kBackLeft_x = -0.875;
        public static final double kBackRight_y = -0.97916;
        public static final double kBackRight_x = -0.875;

        // Wheel Gearing
        public static final double kGearRatio = 12;

        // Speed Constraints
        public static final double kMaxSpeedFeetPerSecond = 5.0;
        public static final double kMaxAccelerationFeetPerSecond = 10.0;

        // PID Constants
        public static final double fl_kP = 0.127;
        public static final double fl_kI = 0;
        public static final double fl_kD = 0;

        public static final double bl_kP = 0.127;
        public static final double bl_kI = 0;
        public static final double bl_kD = 0;

        public static final double fr_kP = 0.127;
        public static final double fr_kI = 0;
        public static final double fr_kD = 0;

        public static final double br_kP = 0.127;
        public static final double br_kI = 0;
        public static final double br_kD = 0;
        
        // Characterization Data
        public static final double kS = 0.495;
        public static final double kV = 2.04;
        public static final double kA = 0.119;
    }

}