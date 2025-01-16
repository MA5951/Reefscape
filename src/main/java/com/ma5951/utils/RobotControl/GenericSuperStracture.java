
package com.ma5951.utils.RobotControl;

import java.util.function.Supplier;

import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GenericSuperStracture {

    protected static Supplier<Pose2d> currentPoseSupplier;
    protected static Supplier<Double> robotVelocitySupplier;
    private static double xTrget;
    private static double yTrget;
    private static double xDis;
    private static double yDis;

    public GenericSuperStracture(Supplier<Pose2d> RobotPoseSupplier , Supplier<Double> robotVelocityVectorSupplier) {
        currentPoseSupplier = RobotPoseSupplier;
    }

    /**
     *  
     * @param redAllianceTarget target when red alliance
     * @param blueAllianceTarget target when blue alliance
     * @return angle to align, should be used with AngleAlignController, return radians absolute to the field (red  wall is 0)
     */
    public static double getSetPointForAline(Pose2d redAllianceTarget , Pose2d blueAllianceTarget) {
        xTrget = DriverStationUtil.getAlliance() == Alliance.Red ? 
            redAllianceTarget.getX() : blueAllianceTarget.getX();

        yTrget = DriverStationUtil.getAlliance() == Alliance.Red ? 
            redAllianceTarget.getY() : blueAllianceTarget.getY();

        xDis = xTrget - currentPoseSupplier.get().getX();
        yDis = yTrget - currentPoseSupplier.get().getY();
        return Math.atan2(yDis , xDis);
    }

    public static boolean isRobotMoving() {
        return robotVelocitySupplier.get() > 0.01; 
    }

    public static double getRobotSpeed() {
        return robotVelocitySupplier.get();
    }
    


    public static boolean isInArea(Translation2d boundingBoxMin, Translation2d boundingBoxMax) { //Rectangle2d rec = new Rectangle2d(boundingBoxMin, boundingBoxMax)
        return currentPoseSupplier.get().getX() >= boundingBoxMin.getX()
              && currentPoseSupplier.get().getY() >= boundingBoxMin.getY()
              && currentPoseSupplier.get().getX() <= boundingBoxMax.getX()
              && currentPoseSupplier.get().getY() <= boundingBoxMax.getY();
    }

    public static boolean hasGamePiece() {
        return false;
    }


}
