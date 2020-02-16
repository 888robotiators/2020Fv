package frc.robot; //kidnap all the robot code

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
// import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    OI oi;

    CANSparkMax shooterMotor;
    CANPIDController pid;
    CANEncoder encoder;


    protected double clicks;
    protected double rpms;
    protected double targetRPM;
    protected double dist;
    protected double SCALAR;
    protected double targetDistZ = 250;

    protected double speedMultiplier;

    protected Indexing index;

    protected boolean state;

    protected UDPReceiver reciever;

    protected double targetZ;
    protected double targetYaw;

    protected Turret turret;

    /**
     * receive
     *
     * @param oi
     * @param p_receiveS
     */
    public Shooter(OI oi, Indexing index, UDPReceiver reciever, Turret turret) {
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_CANID,
                MotorType.kBrushless);
        shooterMotor.setIdleMode(IdleMode.kCoast);
        pid = shooterMotor.getPIDController();
        this.index = index;
        this.oi = oi;
        this.reciever = reciever;
        this.turret = turret;

        encoder = shooterMotor.getEncoder();

        rpms = 0;
        targetRPM = 0;

        state = true;

        pid.setFF(0);
        pid.setP(3.25e-3);
        pid.setI(2e-6);
        pid.setD(0);

        pid.setIZone(500);

        // pid.setIMaxAccum(100, 0);

        pid.setOutputRange(0, 1);

        shooterMotor.setClosedLoopRampRate(0.1);
        shooterMotor.setOpenLoopRampRate(0.1);

    }

    public void shooterTeleopPeriodic() {
        // double[] targetData = receiver.getTarget();
        // SmartDashboard.putNumber("Distance to Target", (double) targetData[2]);
        // clicks = shooterMotor.getEncoder().getPosition();
        // targetDistZ = targetData[2];

        rpms = encoder.getVelocity();
        targetZ = reciever.getTarget()[2];
        targetYaw = reciever.getTarget()[5];
        //double angleToTarget = Math.atan2(reciever.getTarget()[0], reciever.getTarget()[2]);
        
        if (oi.getRightStickButton(RobotMap.JOYSTICK_3D_TRIGGER)) {
            //if (reciever.isVision()) {
                SCALAR = (targetZ < 120) ? 14.5 : (targetZ > 240) ? 13 : 13.5; // change this monstrosity 
                targetRPM = targetZ * SCALAR;
                if (targetYaw > 0.35) {
                    turret.turnTurretMotor(0.5);
                } else if (targetYaw < -0.35) {
                    turret.turnTurretMotor(-0.5);
                } else {
                    turret.turnTurretMotor(0);
                }
                
                //targetRPM = 4000;
            //}
            //else {
            //    targetRPM = 4000;
            // }
            //SmartDashboard.putBoolean("working", true);
        }
        else {
            targetRPM = 0;
            // pid.setIAccum​(0.0);
            // talon.config_kI(0, 0, 0);
            // talon.configClearPositionOnQuadIdx(true, 5);
            // shoot(0);
        }

        shoot(targetRPM);

        // if (targetRPM > rpms) {
        //     shooterMotor.set(1.0);
        // }
        // else if (targetRPM < rpms && targetRPM > 0.0) {
        //     shooterMotor.set(0.1);
        // }
        // else {
        //     shooterMotor.set(0.0);

        // }

        // if (oi.getGamepadButton(RobotMap.X_BUTTON)){
        //     speedMultiplier = -1;
        //     SmartDashboard.putString("Shooter Direction", "Reverse Shoot");
        // }
        // else {
        //     speedMultiplier = 1;
        //     SmartDashboard.putString("Shooter Direction", "Forward Shoot");
        // }
        // if (oi.getGamepadButton(RobotMap.GP_R_BUTTON)) {
        //     shoot(speedMultiplier);
        // }
        // else {
        //     shoot(oi.getGamepadAxis(RobotMap.GP_R_TRIGGER));
        // }

        SmartDashboard.putNumber("Encoder RPMs",
                shooterMotor.getEncoder().getVelocity());
        SmartDashboard.putNumber("Target RPMs", targetRPM);
        // SmartDashboard.putNumber("kP", pid.getP());
        // SmartDashboard.putNumber("kI", pid.getI());
        // SmartDashboard.putNumber("kD", pid.getD());
        // SmartDashboard.putNumber("kFF", pid.getFF());
        // SmartDashboard.putNumber("DistanceAwayFromTarget", receiver.getTarget()[2]);
    }

    public void shoot(double output) {
        pid.setReference(output, ControlType.kVelocity);
        if (output <= 1) {
            pid.setIAccum(0);
        }
    }
}