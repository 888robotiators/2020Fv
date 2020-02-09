package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DigitalInput;

public class Indexing {
    protected TalonSRX bottomBeltIndex;
    protected TalonSRX frontUpperBeltIndex;
    protected TalonSRX frontLowerBeltIndex;
    protected TalonSRX backBeltIndex;

    // 0 is top, 4 is intake
    protected DigitalInput ballPos0;
    protected DigitalInput ballPos1;
    protected DigitalInput ballPos2;
    protected DigitalInput ballPos3;
    protected DigitalInput ballPos4;

    protected OI oi;

    public Indexing(OI oi) {
        bottomBeltIndex = new TalonSRX(RobotMap.INDEX_BOTTOM_BELT_CANID);
        frontUpperBeltIndex = new TalonSRX(
                RobotMap.INDEX_FRONT_UPPER_BELT_CANID);
        frontLowerBeltIndex = new TalonSRX(
                RobotMap.INDEX_FRONT_LOWER_BELT_CANID);
        backBeltIndex = new TalonSRX(RobotMap.INDEX_BACK_BELT_CANID);

        ballPos0 = new DigitalInput(0);
        ballPos1 = new DigitalInput(1);
        ballPos2 = new DigitalInput(2);
        ballPos3 = new DigitalInput(3);
        ballPos4 = new DigitalInput(4);

        this.oi = oi;
    }

    public void indexTeleopPeriodic() {

    }

    public void runBottomBelt(double speed) {
        bottomBeltIndex.set(ControlMode.PercentOutput, speed);
    }

    public void runFrontUpperBelt(double speed) {
        bottomBeltIndex.set(ControlMode.PercentOutput, speed);
    }

    public void runFrontLowerBelt(double speed) {
        bottomBeltIndex.set(ControlMode.PercentOutput, speed);
    }

    public void runBackBelt(double speed) {
        bottomBeltIndex.set(ControlMode.PercentOutput, speed);
    }

    public void loadShooter() {

    }

    public boolean hasBalls() {
        return true;
    }
}