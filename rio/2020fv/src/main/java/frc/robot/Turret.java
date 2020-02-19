package frc.robot;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {

    TalonSRX turretMotor;

    Counter encoder;

    OI oi;

    public Turret(OI oi) {

        turretMotor = new TalonSRX(RobotMap.TURRET_CANID);
        turretMotor.setInverted(true);

        encoder = new Counter(
                new DigitalInput(RobotMap.TURRET_ENCODER_CHANNEL));

        this.oi = oi;
    }

    public void turretTeleopPeriodic() {

        double input = oi.getRightStickAxis(RobotMap.JOYSTICK_3D_Z_AXIS);

        if (oi.getRightStickButton(RobotMap.JOYSTICK_3D_THUMB_BUTTON)
                && Math.abs(input) > 0.1) {
            turnTurretMotor(input);
        }

        else {
            turnTurretMotor(0.0);
        }

        SmartDashboard.putNumber("turret", encoder.get());

    }

    public boolean setAngle(double angle) {
        return false;
    }

    private void turnTurretMotor(double speed) {
        if ((speed > 0 && turretMotor.isFwdLimitSwitchClosed() != 1)
                || (speed < 0 && turretMotor.isRevLimitSwitchClosed() != 1)) {
            turretMotor.set(ControlMode.PercentOutput, speed);
        }
        else {
            turretMotor.set(ControlMode.PercentOutput, 0.0);
        }
    }

}