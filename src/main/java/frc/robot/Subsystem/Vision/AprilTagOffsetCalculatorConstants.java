package frc.robot.Subsystem.Vision;

public class AprilTagOffsetCalculatorConstants {
    // qr code parameters
    public static final double QR_SIZE = 0.10; // in meters (10 cm)

    public static final double QR_Z_OFFSET = 0.30; // in meters (30 cm)

    // camera parameters (in pixels) (get from limelight website) 
    public static final double FX = 800.0;  // focal length X in pixels
    public static final double FY = 800.0;  // focal length Y in pixels
    public static final double CX = 640.0;  // principal point X in pixels
    public static final double CY = 360.0;  // principal point Y in pixels

    // camera distortion coefficients (get from limelight website) will probably be 0
    public static final double[] DIST_COEFFS = {0, 0, 0, 0, 0};

    // camera offset from the center of the robot
    public static final double CAM_X_OFFSET = 0.0;  // offset in X (meters) (left/right)
    public static final double CAM_Y_OFFSET = 0.05; // offset in Y (meters) (forward/backward)
    public static final double CAM_Z_OFFSET = 0.20; // offset in Z (meters) (up/down)

    // camera orientation on the robot
    public static final double CAM_ROLL_DEG  = 0.0;   // Roll angle in degrees
    public static final double CAM_PITCH_DEG = 10.0;  // Pitch angle in degrees (downward is positive)
    public static final double CAM_YAW_DEG   = -5.0;  // Yaw angle in degrees (right is positive)
}