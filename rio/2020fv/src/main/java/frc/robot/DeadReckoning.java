package frc.robot;

import frc.robot.RobotMap;
import frc.robot.DriveTrain;
import disc.data.Position;
import disc.data.Waypoint;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DeadReckoning {

    // Instantiates drive train object
    DriveTrain drive;
    IMU imu;
    UDPReceiver receive;
    WaypointMap map;

    Position pose;

    // Instantiates values for calculating location and speed
    double angle;
    double changeInDistance;
    double changeInEncoderLeft;
    double changeInEncoderRight;
    double changeInHeading;
    double changeInX;
    double changeInY;
    double clickPosX;
    double clickPosY;
    String direction;
    double encoderLeftValue;
    double encoderRightValue;
    double heading;
    double headingValue;
    double lastChangeInEncoderLeft;
    double lastChangeInEncoderRight;
    double lastEncoderLeft;
    double lastEncoderRight;
    double lastHeading;
    double lastTime;
    double PORtoLeft;
    double PORtoRight;
    double posX;
    double posY;
    double speed;
    double time;
    double timePassed;

    boolean calibrated;

    public DeadReckoning(DriveTrain drive, IMU imu, UDPReceiver receive,
            WaypointMap map) {
        // Declares the drive object to be equal to the object passed in by
        // Robot
        this.drive = drive;
        this.imu = imu;
        this.receive = receive;
        this.map = map;

        pose = new Position(0, 0, 0, 0, 0, 0);

        // Resets the encoder values
        reset();
    }

    /**
     * Calculates the location of the robot
     */
    public void updateTracker() {
        // Calls the method to get the most recent encoder data
        updateSensorVals();
        // checkVision();

        // Calculates the change in the encoders and heading since the last time
        // the method was called.
        changeInEncoderLeft = encoderLeftValue - lastEncoderLeft;
        changeInEncoderRight = encoderRightValue - lastEncoderRight;
        changeInHeading = headingValue - lastHeading;

        // Algorithm for when the robot is going forward
        if (changeInEncoderLeft >= 0 && changeInEncoderRight >= 0) {

            changeInDistance = (changeInEncoderLeft + changeInEncoderRight) / 2;

            direction = "forward";

        }

        // Algorithm for when the robot is going backward
        else if (changeInEncoderLeft <= 0 && changeInEncoderRight <= 0) {

            changeInDistance = (changeInEncoderLeft + changeInEncoderRight) / 2;

            direction = "backward";

        }

        // Algorithm for when the robot is spinning clockwise
        else if (changeInEncoderLeft >= 0 && changeInEncoderRight <= 0) {

            // Use a system of equation to find where inside the wheel base the
            // point of rotation is.
            PORtoLeft = (Math.abs(changeInEncoderLeft) * RobotMap.WHEEL_BASE)
                    / (Math.abs(changeInEncoderLeft)
                            + Math.abs(changeInEncoderRight));
            PORtoRight = RobotMap.WHEEL_BASE - PORtoLeft;

            // Find the change in distance based on the law of sines. Use the
            // larger distance from one side to the point of rotation while
            // doing the calculations.
            if (PORtoLeft >= PORtoRight) {

                changeInDistance = Math.sin(changeInHeading)
                        * (PORtoLeft - (RobotMap.WHEEL_BASE / 2))
                        / Math.sin((Math.PI - changeInHeading) / 2);
            }

            else {

                changeInDistance = Math.sin(changeInHeading)
                        * (PORtoRight - (RobotMap.WHEEL_BASE / 2))
                        / Math.sin((Math.PI - changeInHeading) / 2);
            }

            direction = "SCW";

        }

        // Algorithm for when the robot is spinning counterclockwise
        else if (changeInEncoderLeft <= 0 && changeInEncoderRight >= 0) {

            // Use a system of equations to find where inside the wheel base the
            // point of rotation is.
            PORtoLeft = (Math.abs(changeInEncoderLeft) * RobotMap.WHEEL_BASE)
                    / (Math.abs(changeInEncoderLeft)
                            + Math.abs(changeInEncoderRight));
            PORtoRight = RobotMap.WHEEL_BASE - PORtoLeft;

            // Find the change in distance based on the law of sines. Use the
            // larger distance from one side to the point of rotation while
            // doing the calculations.
            if (PORtoLeft >= PORtoRight) {

                changeInDistance = -Math.sin(changeInHeading)
                        * (PORtoLeft - (RobotMap.WHEEL_BASE / 2))
                        / Math.sin((Math.PI - changeInHeading) / 2);
            }

            else {

                changeInDistance = -Math.sin(changeInHeading)
                        * (PORtoRight - (RobotMap.WHEEL_BASE / 2))
                        / Math.sin((Math.PI - changeInHeading) / 2);
            }

            direction = "SCCW";

        }

        // Calculate the angle of change for the bot and the change in heading.
        angle = (headingValue + (changeInHeading / 2));
        heading = RobotMath.modAngleRadians(heading + changeInHeading);

        // Calculates the change in the X and Y directions
        changeInX = changeInDistance * Math.sin(heading);
        changeInY = changeInDistance * Math.cos(heading);

        // Calculates the new position of the robot in inches.
        pose.setX(pose.getX() + changeInX / RobotMap.CLICKS_PER_INCH);
        pose.setY(pose.getY() + changeInY / RobotMap.CLICKS_PER_INCH);
        pose.setHeading(heading);

        // Calculates the speed of the robot in feet per second
        speed = ((Math.sqrt(Math.pow(changeInX, 2) + Math.pow(changeInY, 2))
                / RobotMap.CLICKS_PER_INCH) / 12) / (timePassed);

        pose.setHeading(Math.toDegrees(heading));

    }

    /**
     * Refreshes dashboard values and logs values
     */
    public void updateDashboard() {
        SmartDashboard.putNumber("X Position", pose.getX());
        SmartDashboard.putNumber("Y Position", pose.getY());
        SmartDashboard.putNumber("Heading", pose.getHeading());
        SmartDashboard.putNumber("Left Encoder", encoderLeftValue);
        SmartDashboard.putNumber("Right Encoder", encoderRightValue);
        SmartDashboard.putString("Direction", direction);
    }

    /**
     * Updates n and n-1 encoder value variables.
     */
    private void updateSensorVals() {
        // Saves the change in time
        lastTime = time;
        time = System.currentTimeMillis() / 1000.0;
        /*
         * Calculates the change in time since the last time the method +was
         * called. It should be approximately 20 milliseconds.
         */
        timePassed = time - lastTime;

        // Gets the encoder values from the drive train.
        double[] vals = drive.getEncoders();

        // If the tracker has been calibrated...
        if (calibrated) {
            // ...set the current values to be the last values...
            lastEncoderLeft = encoderLeftValue;
            lastEncoderRight = encoderRightValue;
            lastHeading = headingValue;

        }
        // Otherwise the tracker has not been calibrated...
        else {
            // ...so set the last value to be equal to the current value...
            lastEncoderLeft = vals[0];
            lastEncoderRight = vals[1];
            headingValue = Math.toRadians(RobotMath.modAngleDegrees(heading));
            calibrated = true;
        }

        // ...and set the new encoder values.
        encoderLeftValue = vals[0];
        encoderRightValue = vals[1];
        headingValue = Math
                .toRadians(RobotMath.modAngleDegrees(imu.getHeading()));

    }

    private void checkVision() {
        if (receive.hasVision()) {
            Waypoint visionPose = receive.getTargetPosition();
            if (pose.compareOrthogonal(map.get("Origin"), 6)) {
                pose.setX(0.0); // pose.getX() +
                                // map.get("AllianceTargetZone").getX() -
                                // visionPose.getX());
                pose.setY(0.0); // pose.getY() +
                                // map.get("AllianceTargetZone").getY() -
                                // visionPose.getY());
                SmartDashboard.putBoolean("location", true);
            }
            else SmartDashboard.putBoolean("location", false);
        }
    }

    /**
     * Resets tracking values to 0 or default.
     */
    public void reset() {
        drive.resetEncoderOffset();
        imu.resetHeading();

        encoderLeftValue = 0;
        encoderRightValue = 0;

        // Sets certain location calculating values back to zero
        clickPosX = 0;
        clickPosY = 0;
        direction = "forward";
        lastChangeInEncoderLeft = 0;
        lastChangeInEncoderRight = 0;
        lastEncoderLeft = 0;
        lastEncoderRight = 0;
        lastHeading = 0;
        heading = 0;
        posX = 0;
        posY = 0;

        calibrated = false;
    }

    /**
     * Accessor to get the position logged by the encoders in an array.
     * 
     * @return Returns a double array in format {x, y}
     */
    public Position getPose() {
        return pose;
    }

    /**
     * @return Returns the data for the navigation class
     */
    public String getDirection() {
        return direction;
    }

}