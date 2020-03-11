package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Climber {
    private OI oi;

    private TalonSRX climberMotor;
    private DoubleSolenoid lock;

    private Turret turret;

    private Value isLocked;
    private Value isUnlocked;

    public Climber(OI oi, Turret turret) {
        this.oi = oi;
        this.turret = turret;

        climberMotor = new TalonSRX(20);
        // followerTalon = new TalonSRX(21);

        climberMotor.setInverted(true);

        lock = new DoubleSolenoid(RobotMap.PCM, 2, 3);

        isLocked = Value.kForward;
        isUnlocked = Value.kReverse;

        // followerTalon.follow(climberController);

    }

    public void climberTeleopPeriodic() {
        if(oi.getGamepadAxis(RobotMap.GP_R_TRIGGER) > 0.1 
        || oi.getGamepadAxis(RobotMap.GP_L_TRIGGER) > 0.1 ) {
            if (oi.getGamepadAxis(RobotMap.GP_R_TRIGGER) > 0.1 && readyToClimb()) {
                unlock();
                climberMotor.set(ControlMode.PercentOutput,
                    -oi.getGamepadAxis(RobotMap.GP_R_TRIGGER));
                }
            }
            else if (oi.getGamepadAxis(RobotMap.GP_L_TRIGGER) > 0.1 && readyToClimb()) {
                unlock();
                climberMotor.set(ControlMode.PercentOutput,
                    oi.getGamepadAxis(RobotMap.GP_L_TRIGGER));
                }
            }
            else {
                turret.setAngle(0);
                climberMotor.set(ControlMode.PercentOutput, 0);
            }
        }
        else {
            //lock();
            climberMotor.set(ControlMode.PercentOutput, 0);
            
        }
    

        if (oi.getGamepadButton(RobotMap.GP_R_BUTTON)) {
            lock();
        }
        else if (oi.getGamepadButton(RobotMap.GP_L_BUTTON)) {
            unlock();
        }

        SmartDashboard.putBoolean("Climber Locked", (lock.get().equals(isLocked)));
    }

    public void unlock() {
        lock.set(isUnlocked);
    }

    public void lock() {
        lock.set(isLocked);
    }

    public boolean readyToClimb()
    {
        return(Math.abs(turret.getTurretAngle()) < 7);
    }
}