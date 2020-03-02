package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import disc.data.Position;
import disc.data.Waypoint;
import disc.data.WaypointMap;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {
    private OI oi;
    private DeadReckoning location;
    private Indexing index;
    private Turret turret;
    private WaypointMap map;

    private CANSparkMax shooterMotor;
    private CANPIDController pid;
    private CANEncoder encoder;

    private Waypoint target;
    private Position pose;

    private double targetRPM;
    private double rpms;

<<<<<<< Updated upstream
=======
    private boolean lastUp, lastDown;
    private boolean lastCycleUp, lastCycleDown;

    private int selectedRPM;

    private double distanceToTarget;

    private UDPReceiver receiver;

>>>>>>> Stashed changes
    /**
     * receive
     *
     * @param oi
     * @param p_receiveS
     */
<<<<<<< Updated upstream
    public Shooter(OI oi, DeadReckoning location, Indexing index, Turret turret,
            WaypointMap map) {
=======
    public Shooter(OI oi, DeadReckoning location, Turret turret, WaypointMap map, UDPReceiver receiver) {
>>>>>>> Stashed changes
        this.oi = oi;
        this.location = location;
        this.index = index;
        this.turret = turret;
        this.map = map;
        this.receiver = receiver;

        shooterMotor = new CANSparkMax(RobotMap.SHOOTER_CANID,
                MotorType.kBrushless);
        shooterMotor.setIdleMode(IdleMode.kCoast);
        pid = shooterMotor.getPIDController();

        encoder = shooterMotor.getEncoder();

        rpms = 0;
        targetRPM = 3700;

        pid.setFF(0);
        pid.setP(3.25e-3);
        pid.setI(3e-6);
        pid.setD(0);

        pid.setIZone(500);

        // pid.setIMaxAccum(100, 0);

        pid.setOutputRange(0, 1);

        shooterMotor.setClosedLoopRampRate(0.1);
        shooterMotor.setOpenLoopRampRate(0.1);
    }

    public void shooterTeleopPeriodic() {

        rpms = encoder.getVelocity();
<<<<<<< Updated upstream
        pose = location.getPose();
=======
        //pose = location.getPose();

        if(oi.getRightStickButton(6) && !lastCycleUp) {
            if(selectedRPM == RobotMap.NUM_RPM_SETPOINTS-1) {
                selectedRPM = 0;
                targetRPM = RobotMap.RPM_SETPOINTS[selectedRPM];
            }
            else {
                selectedRPM++;
                targetRPM = RobotMap.RPM_SETPOINTS[selectedRPM];
            }
        }

        if(oi.getRightStickButton(4) && !lastCycleDown) {
            if(selectedRPM == 0) {
                selectedRPM = RobotMap.NUM_RPM_SETPOINTS-1;
                targetRPM = RobotMap.RPM_SETPOINTS[selectedRPM];
            }
            else {
                selectedRPM--;
                targetRPM = RobotMap.RPM_SETPOINTS[selectedRPM];
            }
        }

        if(oi.getGamepadButton(RobotMap.Y_BUTTON)) {
            targetRPM = getRPMs();
        }

>>>>>>> Stashed changes

        if (oi.getRightStickButton(RobotMap.JOYSTICK_3D_TRIGGER)) {
            // shootDistance(map.get("AllianceTargetZone"));
            setShooterOutputVelocity(targetRPM);
        }
        else {
            stop();
        }

        if (oi.getRightStickButton(RobotMap.JOYSTICK_3D_UPPER_LEFT_BUTTON))
            targetRPM += 10;
        if (oi.getRightStickButton(RobotMap.JOYSTICK_3D_LOWER_LEFT_BUTTON))
            targetRPM -= 10;

        SmartDashboard.putNumber("Encoder RPMs", rpms);
        SmartDashboard.putNumber("Target RPMs", targetRPM);
        // SmartDashboard.putNumber("kP", pid.getP());
        // SmartDashboard.putNumber("kI", pid.getI());
        // SmartDashboard.putNumber("kD", pid.getD());
        // SmartDashboard.putNumber("kFF", pid.getFF());
<<<<<<< Updated upstream
        // SmartDashboard.putNumber("DistanceAwayFromTarget",
        // receiver.getTarget()[2]);
=======
        SmartDashboard.putNumber("DistanceAwayFromTarget", receiver.getTarget()[2]);

        lastCycleUp = oi.getRightStickButton(6);
        lastCycleDown = oi.getRightStickButton(4);
>>>>>>> Stashed changes
    }

    /**
     * Runs the shooter at full speed.
     */
    public void shootFullSpeed() {
        shooterMotor.set(1.0);
    }

    /**
     * Runs the shooter at a certain percentage of full speed.
     *
     * @param percentSpeed A percent value between -1.0 and 1.0.
     */
    public void setShooterPercentSpeed(double percentSpeed) {
        shooterMotor.set(percentSpeed);
    }

    /**
     * Causes the object being shot to exit the shooter at a specified velocity.
     *
     * @param velocity The exit velocity in feet per second.
     */
    public void setShooterOutputVelocity(double velocity) {
        targetRPM = velocity;
        pid.setReference(velocity, ControlType.kVelocity);
        if (velocity <= 1) {
            pid.setIAccum(0);
        }
    }

    /**
     * The distance that the shooter should shoot the object
     *
     * @param position target
     */
    public void shootDistance(Waypoint target) {
        turret.setAngle(RobotMath.modAngleDegrees(Math.toDegrees(Math.atan2(
                target.getX() - pose.getX(), target.getX() - pose.getY()))));

        double distanceToTargetWP = Math
                .sqrt(Math.pow(target.getX() - pose.getX(), 2)
                        + Math.pow(target.getX() - pose.getY(), 2));

        targetRPM = distanceToTargetWP * 6.9;

    }

    public void shootDistanceWithVision() {
        pid.setReference(getRPMVision(), ControlType.kVelocity);
    }

    public double getRPMVision() {
        distanceToTarget = receiver.getTarget()[1];
        return distanceToTarget * 6.9;
    }

    /**
     * Calculates the shooter's rotations per minute.
     *
     * @return The number of rotations of the shooter wheel in one minute.
     */
    public double getRPMs() {
        return encoder.getVelocity();
    }

    /**
     * Stop spinning the shooter wheel.
     */
    public void stop() {
        shooterMotor.set(0.0);
    }

    /**
     * Determines if shooter is ready to fire ball
     * 
     * @return True if system is ready to fire a ball
     */
    public boolean readyToFire() {
        if(Math.abs(targetRPM - getRPMs()) < RobotMap.SHOOTER_RPM_TOLERANCE) {
            return true;
        }
        else {
            return false;
        }
    }

}