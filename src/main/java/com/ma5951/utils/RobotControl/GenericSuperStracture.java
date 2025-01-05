
package com.ma5951.utils.RobotControl;

import java.util.function.Supplier;

import com.ma5951.utils.Utils.DriverStationUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class GenericSuperStracture {

    protected Supplier<Pose2d> currentPoseSupplier;
    protected Supplier<Double> robotVelocitySupplier;
    private double xTrget;
    private double yTrget;
    private double xDis;
    private double yDis;

    public GenericSuperStracture(Supplier<Pose2d> RobotPoseSupplier , Supplier<Double> robotVelocityVectorSupplier) {
        currentPoseSupplier = RobotPoseSupplier;
    }

    /**
     *  
     * @param redAllianceTarget target when red alliance
     * @param blueAllianceTarget target when blue alliance
     * @return angle to align, should be used with AngleAlignController, return radians absolute to the field (red  wall is 0)
     */
    public double getSetPointForAline(Pose2d redAllianceTarget , Pose2d blueAllianceTarget) {
        xTrget = DriverStationUtil.getAlliance() == Alliance.Red ? 
            redAllianceTarget.getX() : blueAllianceTarget.getX();

        yTrget = DriverStationUtil.getAlliance() == Alliance.Red ? 
            redAllianceTarget.getY() : blueAllianceTarget.getY();

        xDis = xTrget - currentPoseSupplier.get().getX();
        yDis = yTrget - currentPoseSupplier.get().getY();
        return Math.atan2(yDis , xDis);
    }

    public boolean isRobotMoving() {
        return robotVelocitySupplier.get() > 0.01; //lower value
    }

    public double getRobotSpeed() {
        return robotVelocitySupplier.get();
    }
    
    //has gamepiece

    public boolean isInArea(Translation2d boundingBoxMin, Translation2d boundingBoxMax) { //Rectangle2d rec = new Rectangle2d(boundingBoxMin, boundingBoxMax)
        return currentPoseSupplier.get().getX() >= boundingBoxMin.getX()
              && currentPoseSupplier.get().getY() >= boundingBoxMin.getY()
              && currentPoseSupplier.get().getX() <= boundingBoxMax.getX()
              && currentPoseSupplier.get().getY() <= boundingBoxMax.getY();
    }

    public boolean hasGamePice() {
        return false;
    }


}
