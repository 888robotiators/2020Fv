package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Indexing {

    OI oi;
    Intake intake;
    Shooter shooter;

    TalonSRX funnelMotor;
    TalonSRX lowerFunnelSideIndex;
    TalonSRX lowerFarSideIndex;

    //CANSparkMax upperBeltIndex;
    CANSparkMax loadingBeltIndex;

    // 0 is top, 4 is intake
    DigitalInput bannerPos0;
    DigitalInput bannerPos1;
    DigitalInput bannerPos2;
    DigitalInput bannerPos3;
    DigitalInput bannerPos4;

    boolean ballInPos0;
    boolean ballInPos1;
    boolean ballInPos2;
    boolean ballInPos3;
    boolean ballInPos4;

    boolean loading = false;
    boolean autoLower = true;
    boolean autoUpper = true;

    public Indexing(OI oi, Intake intake, Shooter shooter) {

        this.oi = oi;
        this.intake = intake;
        this.shooter = shooter;

        funnelMotor = new TalonSRX(RobotMap.FUNNEL_MOTOR_CANID);
        lowerFunnelSideIndex = new TalonSRX(
                RobotMap.LOWER_FUNNEL_SIDE_INDEX_CANID);
        lowerFarSideIndex = new TalonSRX(RobotMap.LOWER_FAR_SIDE_INDEX_CANID);

        // upperBeltIndex = new CANSparkMax(RobotMap.UPPER_BELT_CANID,
        //         MotorType.kBrushless);
        loadingBeltIndex = new CANSparkMax(RobotMap.LOADING_BELT_CANID,
                MotorType.kBrushless);

        funnelMotor.setInverted(false);

        //upperBeltIndex.setInverted(true);
        loadingBeltIndex.setInverted(true);

        bannerPos0 = new DigitalInput(0);
        bannerPos1 = new DigitalInput(1);
        bannerPos2 = new DigitalInput(2);
        bannerPos3 = new DigitalInput(3);
        bannerPos4 = new DigitalInput(4);
    }

    /**
     * Runs periodically to control the index
     */
    public void indexManualControls() {

        // Gets joystick values
        double leftGamepadStick = -oi.getGamepadAxis(RobotMap.GP_L_Y_AXIS);
        double rightGamepadStick = -oi.getGamepadAxis(RobotMap.GP_R_Y_AXIS);

        // If the joystick value is outside the deadzone
        if (Math.abs(leftGamepadStick) > RobotMap.JOYSTICK_DEADZONE) {

            // Scales the value
            leftGamepadStick = (Math.abs(leftGamepadStick) - RobotMap.JOYSTICK_DEADZONE)
                    * (1 / (1 - RobotMap.JOYSTICK_DEADZONE))
                    * Math.signum(leftGamepadStick) * RobotMap.INDEX_MAX_SPEED;

            // Runs the motors
            runFunnel(leftGamepadStick);
            runLowerFarSideIndex(leftGamepadStick);
            runLowerFunnelSideIndex(leftGamepadStick);

            autoLower = false;
            
        }

        // Otherwise stop motors
        else {

            runFunnel(0.0);
            runLowerFarSideIndex(0.0);
            runLowerFunnelSideIndex(0.0);            

            autoLower = true;

        }

        // If the joystick value is outside the deadzone
        if (Math.abs(rightGamepadStick) > RobotMap.JOYSTICK_DEADZONE) {

            // Scales the value
            rightGamepadStick = (Math.abs(rightGamepadStick) - RobotMap.JOYSTICK_DEADZONE)
                    * (1 / (1 - RobotMap.JOYSTICK_DEADZONE))
                    * Math.signum(rightGamepadStick) * RobotMap.INDEX_MAX_SPEED;

            // Runs the motors
            runLoadBelt(rightGamepadStick);
            runUpperBelt(rightGamepadStick);

            autoUpper = false;

        }

        // Otherwise stop motors
        else {

            runLoadBelt(0.0);
            runUpperBelt(0.0);
            
            autoUpper = true;

        }

        if (autoLower && autoUpper) {
            bringToTop();
        }

    }

    /**
     * Brings all of the balls in the index to the top of the indexer to be
     * ready to fire.
     */
    public void bringToTop() {

        boolean[] banners = updateBallPoitions();

        if ((!banners[0] && banners[1]) || (!banners[1] && banners[2])
                || (!banners[2] && banners[3])) {
            runLowerFarSideIndex(RobotMap.INDEX_MAX_SPEED);
            runLowerFunnelSideIndex(RobotMap.INDEX_MAX_SPEED);
            runFunnel(RobotMap.INDEX_MAX_SPEED);
        }
        else {
            runLowerFarSideIndex(0.0);
            runLowerFunnelSideIndex(0.0);
            runFunnel(0.0);
        }

        if (!banners[0] && banners[1] || loading) {
            runUpperBelt(RobotMap.INDEX_MAX_SPEED);
            runLoadBelt(RobotMap.INDEX_MAX_SPEED);
        }
        else {
            runUpperBelt(0.0);
            runLoadBelt(0.0);
        }
    }

    public void stopIndexer() {
        runFunnel(0.0);
        runLowerFunnelSideIndex(0.0);
        runLowerFarSideIndex(0.0);
        runUpperBelt(0.0);
        runLoadBelt(0.0);
    }

    public void loadShooter() {
        loading = true;
    }

    public void stopLoadShooter() {
        loading = false;
    }

    public boolean hasBalls() {
        return ballInPos0 || ballInPos1 || ballInPos2 || ballInPos3
                || ballInPos4;
    }

    public boolean readyToFire() {
        return ballInPos0;
    }

    public int getNextBall() {
        if (ballInPos0) return 0;
        if (ballInPos1) return 1;
        if (ballInPos2) return 2;
        if (ballInPos3) return 3;
        if (ballInPos4) return 4;
        return -1;
    }

    public int getNumBalls() {
        int ballsInRobot = 0;

        if (ballInPos0) ballsInRobot++;
        if (ballInPos1) ballsInRobot++;
        if (ballInPos2) ballsInRobot++;
        if (ballInPos3) ballsInRobot++;
        if (ballInPos4) ballsInRobot++;

        return ballsInRobot;
    }

    /**
     * Check to where in the index balls are.
     * 
     * @return A boolean array where each index matches the index of the ball.
     *         True if there is a ball in that index.
     */
    public boolean[] updateBallPoitions() {
        // If there was not a ball in position 0 or the shooter is able to fire,
        // update position 0
        if (!ballInPos0 || shooter.readyToFire()) ballInPos0 = bannerPos0.get();

        // If there was not a ball in position 1 or position 0 is empty, update
        // position
        if (!ballInPos1 || ballInPos0) ballInPos1 = bannerPos1.get();

        // If there was not a ball in position 2 or position 1 is empty, update
        // position
        if (!ballInPos2 || ballInPos1) ballInPos2 = bannerPos2.get();

        // If there was not a ball in position 3 or position 3 is empty, update
        // position
        if (!ballInPos3 || ballInPos2) ballInPos3 = bannerPos3.get();

        // If there was not a ball in position 4 or position 3 is empty, update
        // position
        if (!ballInPos4 || ballInPos3) ballInPos4 = !bannerPos4.get();

        SmartDashboard.putBoolean("ball 0", ballInPos0);
        SmartDashboard.putBoolean("ball 1", ballInPos1);
        SmartDashboard.putBoolean("ball 2", ballInPos2);
        SmartDashboard.putBoolean("ball 3", ballInPos3);
        SmartDashboard.putBoolean("ball 4", ballInPos4);

        return new boolean[] { ballInPos0, ballInPos1, ballInPos2, ballInPos3,
                ballInPos4 };
    }

    /**
     * Runs the funnel motor.
     * 
     * @param speed Percent output for the motor.
     */
    private void runFunnel(double speed) {
        funnelMotor.set(ControlMode.PercentOutput, speed * 0.5);
    }

    /**
     * Runs the lower belts on the funnel side of the index.
     * 
     * @param speed Percent output for the motor.
     */
    private void runLowerFunnelSideIndex(double speed) {
        lowerFunnelSideIndex.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs the lower belts on the side opposite the funnel on the index.
     * 
     * @param speed Percent output for the motor.
     */
    private void runLowerFarSideIndex(double speed) {
        lowerFarSideIndex.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs the upper belts without the loading wheel.
     * 
     * @param speed Percent output for the motor.
     */
    private void runUpperBelt(double speed) {
        //upperBeltIndex.set(speed);
    }

    /**
     * Runs the upper belts with the loading wheel.
     * 
     * @param speed Percent output for the motor.
     */
    private void runLoadBelt(double speed) {
        loadingBeltIndex.set(speed);
    }

}