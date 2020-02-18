package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Climber {
    private OI oi;
    
    private TalonSRX climberMotor;
    private DoubleSolenoid lock;

    public Climber(OI oi) {
        this.oi = oi;
        
        climberMotor = new TalonSRX(20);
        //followerTalon = new TalonSRX(21);

        climberMotor.setInverted(true);
    
        lock = new DoubleSolenoid(RobotMap.PCM, 2, 3);

        //followerTalon.follow(climberController);

    }

    public void climberTeleopPeriodic() {
        if (oi.getGamepadAxis(RobotMap.GP_R_TRIGGER) > 0.1){
            climberMotor.set(ControlMode.PercentOutput, -oi.getGamepadAxis(RobotMap.GP_R_TRIGGER));
        } else if (oi.getGamepadAxis(RobotMap.GP_L_TRIGGER) > 0.1){
            climberMotor.set(ControlMode.PercentOutput, oi.getGamepadAxis(RobotMap.GP_L_TRIGGER));
        }
        else {
            climberMotor.set(ControlMode.PercentOutput, 0);
        }

        if (oi.getGamepadButton(RobotMap.GP_R_BUTTON)) {
            lock.set(Value.kForward);
        }
        else if (oi.getGamepadButton(RobotMap.GP_L_BUTTON)) {
            lock.set(Value.kReverse);
        }
    }
}