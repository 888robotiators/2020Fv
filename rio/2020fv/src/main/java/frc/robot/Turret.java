package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Spark;

public class Turret {
    protected Spark turretMotor;
    protected DigitalInput middleBannerSensor;

    protected DigitalInput leftLimitSwitch;
    protected DigitalInput rightLimitSwitch;

    protected DigitalInput encoder;

    protected OI oi;

    protected boolean lastEncoder;
    protected boolean currentEncoder;

    protected int encoderCounter;

    public Turret(OI oi) {
        turretMotor = new Spark(0);

        encoder = new DigitalInput(0);

        middleBannerSensor = new DigitalInput(0);

        leftLimitSwitch = new DigitalInput(0);
        rightLimitSwitch = new DigitalInput(0);

        this.oi = oi;

        lastEncoder = false;
    }

    public void turretTeleopPeriodic() {
        if (oi.getRightStickAxis(3) > 0.1) {

        }
    }

    public boolean turnTurret(double angle) {
        return false;
    }

    public void encoderThings() {
        currentEncoder = encoder.get();
        if (currentEncoder && !lastEncoder) {
            encoderCounter++;
        }

        lastEncoder = currentEncoder;
    }
}