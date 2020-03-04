package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Intake {

    private OI oi;

    private DoubleSolenoid intakeFlipper;
    private TalonSRX intakeMotor;

    private boolean flipButtonLast = false;
    private boolean flipButton = false;

    private boolean isRunning = false;

    public Intake(OI oi) {
        intakeFlipper = new DoubleSolenoid(RobotMap.PCM,
                RobotMap.DS_FORWARD_CHANNEL, RobotMap.DS_REVERSE_CHANNEL);
        intakeMotor = new TalonSRX(RobotMap.INTAKE_CANID);

        intakeMotor.setInverted(true);

        this.oi = oi;

        //intakeIn();
    }
    
    public DoubleSolenoid getFlipper() {
        return intakeFlipper;
    }
    /**
     * Teleop controls for the intake.
     */
    public void intakeTeleopPeriodic() {
        if (oi.getLeftStickButton(RobotMap.JOYSTICK_TRIGGER)) {
            if (oi.getLeftStickButton(RobotMap.JOYSTICK_LEFT_BUTTON)) {
                intakeReverse();
            }
            else {
                intakeIn();
            }
        }
        else {
            intakeStop();
        }

        flipButton = oi.getLeftStickButton(RobotMap.JOYSTICK_CENTER_BUTTON);

        flipIntake(flipButton && !flipButtonLast);

        flipButtonLast = flipButton;

        if (intakeFlipper.get().equals(DoubleSolenoid.Value.kForward)) {
            intakeStop();
        }else if (intakeFlipper.get().equals(DoubleSolenoid.Value.kReverse)) {
            intakeIn();
        }

    }

    /**
     * Toggles the state of the intake arm. If the arm is out, it will be
     * flipped in, and if it is in, it will be flipped out.
     * 
     * @param flip True to toggle
     */
    public void flipIntake(boolean flip) {
        if (flip) {

            if (intakeFlipper.get().equals(DoubleSolenoid.Value.kReverse)) {
                intakeFlipper.set(DoubleSolenoid.Value.kForward);
            }

            else {
                intakeFlipper.set(DoubleSolenoid.Value.kReverse);
            }
        }
    }

    /**
     * Runs the intake wheels in.
     */
    public void intakeIn() {
        intakeMotor.set(ControlMode.PercentOutput, 1.0);
        isRunning = true;
    }

    /**
     * Runs the intake wheels out.
     */
    public void intakeReverse() {
        intakeMotor.set(ControlMode.PercentOutput, -1.0);
        isRunning = true;
    }

    /**
     * Stops running the intake wheels.
     */
    public void intakeStop() {
        intakeMotor.set(ControlMode.PercentOutput, 0);
        isRunning = false;
    }

    /**
     * Flips the intake arms out.
     */
    public void flipDown() {
        if (intakeFlipper.get().equals(DoubleSolenoid.Value.kReverse))
            intakeFlipper.set(DoubleSolenoid.Value.kForward);
    }

    /**
     * Flips the intake arms in.
     */
    public void flipUp() {
        if (intakeFlipper.get().equals(DoubleSolenoid.Value.kForward))
            intakeFlipper.set(DoubleSolenoid.Value.kReverse);
    }

    /**
     * Gets whether or not the intake wheels are running.
     * 
     * @return True if the wheels are running.
     */
    public boolean isRunning() {
        return isRunning;
    }

    /**
     * Gets the state of the intake arms.
     * 
     * @return True if the intake arms are flipped out.
     */
    public boolean isFlippedOut() {
        return intakeFlipper.get().equals(DoubleSolenoid.Value.kForward);
    }

}