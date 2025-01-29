package frc.robot.Subsystem.Vision;

public class AprilTagOffsetCalculatorConstants {
    // QR code parameters
    public static final double QR_SIZE = 0.1651; // in meters (16.51 cm)
    public static final double QR_Z_OFFSET = 0.29845; // in meters (29.845 cm)

    // Camera Field of View in degrees
    public static final double FOV_X_DEG = 80.0; // Horizontal FOV
    public static final double FOV_Y_DEG = 56.0; // Vertical FOV

    // Image resolution in pixels
    public static final int IMAGE_WIDTH = 1280; // Example: 1280 pixels width
    public static final int IMAGE_HEIGHT = 720; // Example: 720 pixels height

    // Camera Intrinsic Parameters (computed based on FOV and image resolution)
    public static final double FX;
    public static final double FY;
    public static final double CX;
    public static final double CY;

    static {
        // Convert FOV from degrees to radians
        double fovXRad = Math.toRadians(FOV_X_DEG);
        double fovYRad = Math.toRadians(FOV_Y_DEG);

        // Compute focal lengths in pixels based on FOV and image size
        FX = IMAGE_WIDTH / (2.0 * Math.tan(fovXRad / 2.0));
        FY = IMAGE_HEIGHT / (2.0 * Math.tan(fovYRad / 2.0));

        // Principal point (assuming it's at the center of the image)
        CX = IMAGE_WIDTH / 2.0;
        CY = IMAGE_HEIGHT / 2.0;
    }

    // Camera distortion coefficients (assuming no distortion)
    public static final double[] DIST_COEFFS = {0, 0, 0, 0, 0};

    // Camera offset from the center of the robot (in meters)
    public static final double CAM_X_OFFSET = 0.0;  // meters (left/right)
    public static final double CAM_Y_OFFSET = 0.05; // meters (forward/backward)
    public static final double CAM_Z_OFFSET = 0.20; // meters (up/down)

    // Camera orientation on the robot (in degrees)
    public static final double CAM_ROLL_DEG  = 0.0;   // Roll angle
    public static final double CAM_PITCH_DEG = 10.0;  // Pitch angle (downward is positive)
    public static final double CAM_YAW_DEG   = -5.0;  // Yaw angle (right is positive)
}
