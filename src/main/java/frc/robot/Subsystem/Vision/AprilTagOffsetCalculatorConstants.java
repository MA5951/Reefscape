package frc.robot.Subsystem.Vision;

public class AprilTagOffsetCalculatorConstants {
    // April Tag Parameters
    public static final double QR_SIZE = 0.1651; // in meters (16.51 cm)
    public static final double QR_Z_OFFSET = 0.29845; // in meters (29.845 cm)

    // Camera Field of View in Degrees
    public static final double FOV_X_DEG = 80.0; // Horizontal FOV
    public static final double FOV_Y_DEG = 56.0; // Vertical FOV
    
    // Image Resolutions and Frame Rates
    // Mode 1: 1280x800 at 120 FPS
    public static final int IMAGE_WIDTH_1280x800 = 1280; // pixels
    public static final int IMAGE_HEIGHT_1280x800 = 800; // pixels
    public static final int FRAME_RATE_1280x800 = 120;   // FPS

    // Mode 2: 640x480 at 240 FPS
    public static final int IMAGE_WIDTH_640x480 = 640;   // pixels
    public static final int IMAGE_HEIGHT_640x480 = 480;  // pixels
    public static final int FRAME_RATE_640x480 = 240;    // FPS

    // Select Active Mode
    // Choose the active mode by setting one of the following flags to true
    public static final boolean USE_MODE_1280x800 = true;
    public static final boolean USE_MODE_640x480 = false;

    // Active Image Resolution
    public static final int IMAGE_WIDTH;
    public static final int IMAGE_HEIGHT;

    // Principal Point Coordinates in Pixels
    public static final double CX; // Principal point X-coordinate
    public static final double CY; // Principal point Y-coordinate

    // Camera Intrinsic Parameters (Computed Dynamically)
    public static final double FX;
    public static final double FY;

    static {
        // Determine active image resolution based on selected mode
        if (USE_MODE_1280x800) {
            IMAGE_WIDTH = IMAGE_WIDTH_1280x800;
            IMAGE_HEIGHT = IMAGE_HEIGHT_1280x800;
        } else if (USE_MODE_640x480) {
            IMAGE_WIDTH = IMAGE_WIDTH_640x480;
            IMAGE_HEIGHT = IMAGE_HEIGHT_640x480;
        } else {
            throw new IllegalStateException("No active camera mode selected in AprilTagOffsetCalculatorConstants.");
        }

        // Compute principal point assuming it's at the image center
        CX = IMAGE_WIDTH / 2.0;
        CY = IMAGE_HEIGHT / 2.0;

        // Convert FOV from degrees to radians
        double fovXRad = Math.toRadians(FOV_X_DEG);
        double fovYRad = Math.toRadians(FOV_Y_DEG);

        // Compute focal lengths in pixels based on FOV and image size
        FX = IMAGE_WIDTH / (2.0 * Math.tan(fovXRad / 2.0));
        FY = IMAGE_HEIGHT / (2.0 * Math.tan(fovYRad / 2.0));
    }

    // Camera Distortion Coefficients (Assumed Zero)
    public static final double[] DIST_COEFFS = {0, 0, 0, 0, 0};

    // Camera Extrinsic Parameters (Relative to Robot Frame)
    // Camera position offsets from robot center (in meters) (z axis 0 is ground)
    public static final double CAM_X_OFFSET = 0.0;  // meters (left/right)
    public static final double CAM_Y_OFFSET = 0.05; // meters (forward/backward)
    public static final double CAM_Z_OFFSET = 0.20; // meters (up/down)

    // Camera orientation angles relative to robot frame (in degrees)
    public static final double CAM_ROLL_DEG  = 0.0;   // Roll angle
    public static final double CAM_PITCH_DEG = 10.0;  // Pitch angle (downward is positive)
    public static final double CAM_YAW_DEG   = -5.0;  // Yaw angle (right is positive)
}
