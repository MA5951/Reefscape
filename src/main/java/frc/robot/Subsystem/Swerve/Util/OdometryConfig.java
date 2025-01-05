
package frc.robot.Subsystem.Swerve.Util;

public class OdometryConfig {

    public boolean updateInCollision;
    public boolean updateInSkid;
    public double skidRatio;
    public double collisionForce;
    public Boolean updateWhenStuck;
    public double currentWhenStck;

    public OdometryConfig(
        boolean UpdateInCollision , 
        boolean UpdateInSkid ,
        double SkidRatio , 
        double CollisionForce ,
        Boolean UpdateWhenStuck,
        double CurrentWhenStck) {
        updateInCollision = UpdateInCollision;
        updateInSkid = UpdateInSkid;
        collisionForce = CollisionForce;
        skidRatio = SkidRatio;
        updateWhenStuck = UpdateWhenStuck;
        currentWhenStck = CurrentWhenStck;
    }

}
