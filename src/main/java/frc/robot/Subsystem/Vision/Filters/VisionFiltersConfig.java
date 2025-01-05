
package frc.robot.Subsystem.Vision.Filters;


import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class VisionFiltersConfig {

    private final static double FIELD_TOLERANCE = 0.5;
    public final static double SPEED_FOR_GYRO_RESET = 0.05;
    public final static double AMBIGUITY_FOR_GYRO_RESET = 0.5;
    public final static double VISION_VELOCITY_TOLERANCE = 0.35;

    public final boolean updateInAuto;
    public final ChassisSpeeds robotUpdateSpeed;
    public final Rectangle2d fieldRectangle;
    public final Rectangle2d[] fieldObstaclesRectangles;
    public final double visionToOdometry;
    public final boolean visionToOdometryInTeleop;
    public final double maxVelocityForVisionVelocityFilter;


    public VisionFiltersConfig(
        boolean UpdateInAuto,
        ChassisSpeeds RobotUpdateSpeed,
        Rectangle2d FieldRectangle,
        Rectangle2d[] FieldObstaclesRectangles,
        double VisionToOdometry,
        boolean VisionToOdometryInTeleop,
        double MaxVelocityForVisionVelocityFilter
    ) {
        updateInAuto = UpdateInAuto;
        robotUpdateSpeed = RobotUpdateSpeed;
        fieldRectangle = new Rectangle2d(new Translation2d(-FIELD_TOLERANCE , -FIELD_TOLERANCE), new Translation2d(FieldRectangle.getXWidth() + FIELD_TOLERANCE , FieldRectangle.getYWidth() + FIELD_TOLERANCE));
        fieldObstaclesRectangles = FieldObstaclesRectangles;
        visionToOdometry = VisionToOdometry;
        visionToOdometryInTeleop = VisionToOdometryInTeleop;
        maxVelocityForVisionVelocityFilter = MaxVelocityForVisionVelocityFilter;
    }

    public VisionFiltersConfig() {
        this(false, new ChassisSpeeds(), new Rectangle2d(null, null), new Rectangle2d[1], 0d, false , 0);
    }

    


}
