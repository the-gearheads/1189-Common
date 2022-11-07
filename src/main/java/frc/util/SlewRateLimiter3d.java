package frc.util;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class SlewRateLimiter3d {

    private SlewRateLimiter xSlewRateLimiter;
    private SlewRateLimiter ySlewRateLimiter;
    private SlewRateLimiter rotSlewRateLimiter;
    private double prevRot;

    public SlewRateLimiter3d(double xSlewLimit, double ySlewLimit){
        this.xSlewRateLimiter=new SlewRateLimiter(xSlewLimit);
        this.ySlewRateLimiter=new SlewRateLimiter(ySlewLimit);
    }
    public Pose3d calculate(Pose3d pos){
        return new Pose3d(xSlewRateLimiter.calculate(pos.getX()),
                          ySlewRateLimiter.calculate(pos.getY()),
                          pos.getZ(),
                          pos.getRotation());
    }
}
