
package frc.robot.Subsystem.Vision;

import com.ma5951.utils.Utils.ConvUtil;

import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.RobotConstants;
import frc.robot.Subsystem.Vision.Filters.VisionFiltersConfig;

public class VisionConstants {

    public final static Translation3d ROBOT_TO_CAMERA_XYZ = new Translation3d(0.1, 0, 0.5);
    public final static Rotation3d ROBOT_TO_CAMERA_ROTATION = new Rotation3d(0, Math.toRadians(-15), 0);
    public final static Transform3d ROBOT_TO_CAMERA = new Transform3d(ROBOT_TO_CAMERA_XYZ, ROBOT_TO_CAMERA_ROTATION);

    public final static double[] TAG_HIGHTS = {
            1.22, 1.22, 1.32, 1.32, 1.22,
            1.22, 1.455, 1.455, 1.22, 1.22,
            1.21, 1.21, 1.21, 1.21, 1.21, 1.21
    };

    public final static VisionFiltersConfig TELEOP_FILTERS_CONFIG = new VisionFiltersConfig(
            true,
            new ChassisSpeeds(4, 4, ConvUtil.DegreesToRadians(720)),
            new Rectangle2d(new Translation2d(), RobotConstants.FieldFarCorner),
            null,
            1,
            false,
            2.5);

    public final static VisionFiltersConfig AUTO_FILTERS_CONFIG = TELEOP_FILTERS_CONFIG;

    public static final VisionIO getVisionIO() {
        return new VisionSim();
    }
}
