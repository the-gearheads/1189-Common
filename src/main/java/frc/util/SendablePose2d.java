package frc.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

public class SendablePose2d implements Sendable{
    private Pose2d pose;

    public SendablePose2d(Pose2d pose){
        this.pose = pose;
    }
    
    @Override
    public void initSendable(SendableBuilder builder) {
      builder.setSmartDashboardType("Pose2d");
      builder.addDoubleProperty("X", pose::getX, (x)->{});
      builder.addDoubleProperty("Y", pose::getY, (y)->{});
      builder.addDoubleProperty("Angle", ()->pose.getRotation().getDegrees(), (angle)->{});
    }
}
