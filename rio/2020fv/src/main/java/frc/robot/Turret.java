package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Turret {

    TalonSRX turretMotor;

    OI oi;

    double turretAngle;

    int encoderCount;

    boolean leftLimitSwitch = false;
    boolean rightLimitSwitch = false;

    public Turret(OI oi) {

        this.oi = oi;

        turretMotor = new TalonSRX(RobotMap.TURRET_CANID);
        turretMotor.setInverted(true);

        turretMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turretMotor.setSelectedSensorPosition(0);
        turretAngle = -30;
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

        SmartDashboard.putNumber("turret", encoderCount);

    }

    public boolean setAngle(double angle) {
        if (Math.abs(turretAngle - angle) < RobotMap.TURRET_ANGLE_TOLERANCE) {
            turnTurretMotor(0.0);
            return true;
        }

        if (turretAngle > angle) {
            turnTurretMotor(-1.0);
        }

        if (turretAngle < angle) {
            turnTurretMotor(1.0);
        }

        return false;
    }

    private void turnTurretMotor(double speed) {

        leftLimitSwitch = turretMotor.isFwdLimitSwitchClosed() == 1;
        rightLimitSwitch = turretMotor.isRevLimitSwitchClosed() == 1;

        encoderCount = -turretMotor.getSelectedSensorPosition();

        SmartDashboard.putBoolean("Turret Left Limit", leftLimitSwitch);
        SmartDashboard.putBoolean("Turret Right Limit", rightLimitSwitch);

        if ((speed > 0 && !leftLimitSwitch) || (speed < 0 && !rightLimitSwitch)) {
            turretMotor.set(ControlMode.PercentOutput, speed);
        }
        else {
            turretMotor.set(ControlMode.PercentOutput, 0.0);
        }

        if (rightLimitSwitch) {
            turretMotor.setSelectedSensorPosition(6750);
        }

        if (leftLimitSwitch) {
            turretMotor.setSelectedSensorPosition(-6750);
        }

        turretAngle = encoderCount / RobotMap.TURRET_CLICKS_PER_DEGREE;
        SmartDashboard.putNumber("Turret angle", turretAngle);
        
    }

}