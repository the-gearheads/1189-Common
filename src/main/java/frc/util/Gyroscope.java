package frc.util;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;

public class Gyroscope {
    private AHRS ahrs;
    private boolean isInverted;
    public Gyroscope(AHRS ahrs){
        this(ahrs, false);
    }

    public Gyroscope(AHRS ahrs, boolean isInverted){
        this.ahrs = ahrs;
        this.isInverted = isInverted;
    }
    public void setInverted(boolean isInverted){
        this.isInverted = false;
    }
    public Rotation2d getRotation2d(){
        double direction = isInverted ? 1 : -1;
        return new Rotation2d(direction * ahrs.getRotation2d().getRadians());
    }
    public void setRotation2d(Rotation2d newRotation2d){
        double direction = isInverted ? -1 : 1;
        double newAngle = newRotation2d.getDegrees() * direction;
        ahrs.zeroYaw();
        ahrs.setAngleAdjustment(newAngle);
    }
    public double getContinuousAngle(){
        double direction = isInverted ? -1 : 1;
        return direction * ahrs.getAngle();
    }
}
