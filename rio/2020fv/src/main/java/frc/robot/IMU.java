package frc.robot;

import edu.wpi.first.wpilibj.SPI;

import com.kauailabs.navx.frc.AHRS;

public class IMU {

    AHRS navX;

    double headingOffset;

    public IMU() {

        navX = new AHRS(SPI.Port.kMXP);

    }

    public void resetHeading() {

        headingOffset = RobotMath.modAngleDegrees(navX.getYaw());
    }

    public double getHeading() {
        return RobotMath.modAngleDegrees(navX.getYaw() - headingOffset);
    }

}