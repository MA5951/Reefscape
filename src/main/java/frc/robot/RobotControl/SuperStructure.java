
package frc.robot.RobotControl;


import com.ma5951.utils.RobotControl.GenericSuperStracture;

import frc.robot.Subsystem.PoseEstimation.PoseEstimator;
import frc.robot.Subsystem.Swerve.SwerveSubsystem;

public class SuperStructure extends GenericSuperStracture{



    public SuperStructure() {
        super(() -> PoseEstimator.getInstance().getEstimatedRobotPose(), () -> SwerveSubsystem.getInstance().getVelocityVector());
    }

    public void update() {

    }


}
