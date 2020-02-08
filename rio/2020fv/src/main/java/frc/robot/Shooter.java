package frc.robot; //kidnap all the robot code

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANPIDController;
// import edu.wpi.first.wpilibj.GearTooth;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    protected OI oi;
    // protected GearTooth gt;

    // protected TalonSRX talon;

    protected CANSparkMax shooterMotor;
    protected CANPIDController pid;

    protected double clicks;
    protected double rpms;
    protected double targetRPM;
    protected double dist;
    protected final double SCALAR = 13.5;
    protected double targetDistZ = 250;

    protected UDPReceiver receiver;

    protected double speedMultiplier;

    protected Indexing index;

    protected boolean state;

    /**
     * receive
     *
     * @param oi
     * @param p_receiveS
     */
    public Shooter(OI oi, UDPReceiver receiver, Indexing index) {
        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_CANID,
                MotorType.kBrushless);
        shooterMotor.setIdleMode(IdleMode.kCoast);
        pid = shooterMotor.getPIDController(); // new
                                               // CANPIDController(sparkMax);

        // talon = new TalonSRX(20);

        // gt = new GearTooth(RobotMap.SHOOTER_ENCODER);
        this.index = index;
        this.oi = oi;
        this.receiver = receiver;

        rpms = 0;
        targetRPM = 0;

        state = true;

        pid.setFF(0);
        pid.setP(1e-3);
        pid.setI(1.75e-6);
        pid.setD(0);

        pid.setIZone(1000);

        // pid.setIMaxAccum(100, 0);

        pid.setOutputRange(0, 1);

        shooterMotor.setClosedLoopRampRate(0.1);
        shooterMotor.setOpenLoopRampRate(0.1);

        // talon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        // 0, 0);
        // talon.setSensorPhase(false);
        // talon.setSelectedSensorPosition(0);

        // talon.configNominalOutputForward(0);
        // talon.configNominalOutputReverse(0);
        // talon.configPeakOutputForward(1);
        // talon.configPeakOutputReverse(-1);

        // //double f = 0.0324;
        // //P I D eez nuts
        // talon.config_kF(0, 0.1, 0);
        // talon.config_kP(0, 0.285, 0);
        // talon.config_kI(0, 0.00035, 0);
        // talon.config_kD(0, 0, 0);

        // talon.configMaxIntegralAccumulator(0, 100);

    }

    public void shooterTeleopPeriodic() {
        double[] targetData = receiver.getTarget();
        SmartDashboard.putNumber("Distance to Target", (double) targetData[2]);
        clicks = shooterMotor.getEncoder().getPosition();
        targetDistZ = targetData[2];

        if (oi.getRightStickButton(RobotMap.JOYSTICK_TRIGGER)) {
            targetRPM = targetDistZ * SCALAR;
            SmartDashboard.putBoolean("working", true);
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
        // shoot(1.0);
        // }
        // // else if (targetRPM < rpms && targetRPM > 0.0) {
        // // shoot(0.1);
        // // }
        // else {
        // shoot(0.0);

        // }

        // // if (oi.getGamepadButton(RobotMap.X_BUTTON)){
        // // speedMultiplier = -1;
        // // SmartDashboard.putString("Shooter Direction", "Reverse Shoot");
        // // }
        // // else {
        // // speedMultiplier = 1;
        // // SmartDashboard.putString("Shooter Direction", "Forward Shoot");
        // // }
        // // if (oi.getGamepadButton(RobotMap.GP_R_BUTTON)) {
        // // shoot(speedMultiplier);
        // // }
        // // else {
        // // shoot(oi.getGamepadAxis(RobotMap.GP_R_TRIGGER));
        // // }
        SmartDashboard.putNumber("Encoder RPMs",
                shooterMotor.getEncoder().getVelocity());
        // SmartDashboard.putNumber("Shooter RPMs", rpms);

        // SmartDashboard.putNumber("Teeth Counted", clicks);
        SmartDashboard.putNumber("Target RPMs", targetRPM);
        // SmartDashboard.putNumber("kP", pid.getP());
        // SmartDashboard.putNumber("kI", pid.getI());
        // SmartDashboard.putNumber("kD", pid.getD());
        // SmartDashboard.putNumber("kFF", pid.getFF());
        // SmartDashboard.putNumber("DistanceAwayFromTarget",
        // receiver.getTarget()[2]);
    }

    public void shoot(double output) {

        pid.setReference(output, ControlType.kVelocity);
        if (output <= 1) {
            pid.setIAccum(0);
        }
    }
}