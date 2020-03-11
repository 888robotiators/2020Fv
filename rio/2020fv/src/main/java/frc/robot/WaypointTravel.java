package frc.robot;

import disc.data.Position;

import disc.data.Waypoint;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class WaypointTravel {

    DriveTrain drive;
    DeadReckoning location;

    Position pose;

    double speedAdjustment;
    double leftThrust;
    double rightThrust;

    int state = 0;

    /**
     * Constructor for guidance object.
     * 
     * @param p_drive Global {@link DriveTrain} object.
     * @param p_location Global {@link DeadReckoning} object.
     */
    public WaypointTravel(DriveTrain p_drive, DeadReckoning p_location) {
        drive = p_drive;
        location = p_location;
    }

    /**
     * Causes the robot to travel to a waypoint.
     * 
     * @param desiredX Final x position.
     * @param desiredY Final y position.
     * @param desiredAngle Final heading.
     * @param speed The minimum travel speed.
     * @return True if the robot has arrived at the point.
     */
    public boolean goToWaypoint(Waypoint destination, double speed) {

        double desiredX = destination.getX();
        double desiredY = destination.getY();
        double desiredAngle = RobotMath.modAngleDegrees(destination.getHeading());
        SmartDashboard.putNumber("dtheta", desiredAngle);
        

        // Finds where the robot is on the field
        pose = location.getPose();

        // Initializes the arrived boolean to false
        boolean arrived = false;

        // Finds the difference between the current and desired headings
        double headingDifference = findDesiresdAngle(desiredX, desiredY);

        // if (headingDifference > 180) {
        // headingDifference -= 360;
        // }

        SmartDashboard.putNumber("hd", headingDifference);
        SmartDashboard.putNumber("State", state);

        switch (state) {

            case 0:
                if (Math.abs(desiredX) < 0.1 && Math.abs(desiredY) < 0.1) {
                    state = 3;
                }
                else {
                    state = 2;
                }
                drive.brake();
                break;

            case 1:
                // Angle to heading for point

                // If the difference in the actual and desired headings is
                // greater than 180 degrees...
                if (headingDifference > 180) {
                    // ...go the other way around the circle.
                    headingDifference -= 360;
                }

                double course = headingDifference;

                if (speed < 0) {
                    course = RobotMath.modAngleDegrees(headingDifference + 180);
                }

                // If the robot is not within 4 degrees of its target heading...
                if (Math.abs(course) > RobotMap.ANGLE_TOLERENCE) {
                    double[] rotationSpeed = turn(course, speed);
                    drive.move(rotationSpeed[0], rotationSpeed[1]);
                }

                // Otherwise...
                else {
                    // ...stop moving and go to the next step.
                    drive.move(0.0, 0.0);
                    state = 2;
                }
                break;

            case 2:

                // Travel to point

                if (!((Math.abs(
                        desiredX - pose.getX()) < RobotMap.POSITION_TOLERENCE)
                        && (Math.abs(desiredY - pose
                                .getY()) < RobotMap.POSITION_TOLERENCE))) {
                    double[] speeds = move(headingDifference, speed);
                    drive.move(-speeds[1], -speeds[0]);

                }

                else {
                    drive.move(0.0, 0.0);
                    state = 4;  // FIX THIS TO ANGLE - should go to state 3 to
                                // when complete with driving
                }

                break;

            case 3:

                // Turn to final angle

                // If the robot is not within --- degs of its target
                // heading...
                if (Math.abs(RobotMath.modAngleDegrees(
                        desiredAngle - pose.getHeading())) > 2) {

                    SmartDashboard.putNumber("test",
                            Math.abs(RobotMath.modAngleDegrees(
                                    desiredAngle - pose.getHeading())));

                    // ...go to that heading.
                    double[] rotationSpeed = turn(RobotMath.modAngleDegrees(
                            desiredAngle - pose.getHeading()), speed);

                    drive.move(rotationSpeed[0], rotationSpeed[1]);

                }
                // Otherwise...
                else {
                    // ...stop moving and go to the next step.
                    drive.brake();
                    drive.move(0.0, 0.0);
                    state = 4;
                }

                break;

            case 4:
                // Set the arrived boolean to true and reset the state to zero
                arrived = true;
                state = 0;
                break;

            default:
                break;
        }

        // Return whether or not the robot has arrived
        return arrived;
    }

    /**
     * Have to robot travel to a point
     * 
     * @param angle The angle at which the robot should travel.
     * @param speed The speed at which the robot should travel.
     * @return The speed for the left and right sides of the robot.
     */
    private double[] move(double angle, double speed) {
        double heading = RobotMath.modAngleDegrees(pose.getHeading());

        SmartDashboard.putNumber("speed", speed);

        double headingDifference = angle;
        if (headingDifference > 180) {
            headingDifference -= 360;
        }

        // Calculates the adjustment based on how much the robot needs to turn
        speedAdjustment = Math.max(0.0,
                Math.min(0.2, (Math.abs(headingDifference) / 45)));

        SmartDashboard.putNumber("proportion", speedAdjustment);

        // If the robot is going forward...
        if (speed > 0) {

            // If the heading is to the right of the robot's current heading...
            if (angle > 0) {

                /*
                 * If the speed plus the adjustment for the left side would be
                 * slower than the max speed add the adjustments to the left
                 * side. Otherwise, subtract the adjustments from the right
                 * side.
                 */

                leftThrust = ((speed + speedAdjustment) <= 1)
                        ? speed + speedAdjustment : 1.0;

                rightThrust = ((speed + speedAdjustment) <= 1) ? speed
                        : speed - (speedAdjustment - (1 - speed));

            }

            // If the heading is to the left of the robot's current heading...
            else if (angle < 0) {

                /*
                 * If the speed plus the adjustment for the right side would be
                 * slower than the max speed add the adjustments to the right
                 * side. Otherwise, subtract the adjustments from the left side.
                 */

                leftThrust = ((speed + speedAdjustment) <= 1) ? speed
                        : speed - (speedAdjustment - (1 - speed));

                rightThrust = ((speed + speedAdjustment) <= 1)
                        ? speed + speedAdjustment : 1.0;

            }
        }

        // If the robot is going backward...
        else if (speed < 0) {
            double course = RobotMath.modAngleDegrees(heading + 180);
            if (course > 180) course -= 360;

            angle -= 180;

            SmartDashboard.putNumber("course", course);

            // If the heading is to the right of the robot's current heading...
            if (angle < 0) {

                /*
                 * If the speed plus the adjustment for the left side would be
                 * slower than the max speed add the adjustments to the left
                 * side. Otherwise, subtract the adjustments from the right
                 * side.
                 */

                leftThrust = ((speed - speedAdjustment) >= -1)
                        ? speed - speedAdjustment : -1.0;

                rightThrust = ((speed - speedAdjustment) >= -1) ? speed
                        : speed + speedAdjustment + (-1 - speed);

            }

            // If the heading is to the right of the robot's current heading...
            else if (angle > 0) {

                /*
                 * If the speed plus the adjustment for the right side would be
                 * slower than the max speed add the adjustments to the right
                 * side. Otherwise, subtract the adjustments from the left side.
                 */

                leftThrust = ((speed - speedAdjustment) >= -1) ? speed
                        : speed + speedAdjustment + (-1 - speed);

                rightThrust = ((speed - speedAdjustment) >= -1)
                        ? speed - speedAdjustment : -1.0;

            }
        }

        // Otherwise make no adjustments
        else {
            leftThrust = 0.0;
            rightThrust = 0.0;
        }

        SmartDashboard.putNumber("left auto", leftThrust);
        SmartDashboard.putNumber("right auto", rightThrust);

        return new double[] { leftThrust, rightThrust };
    }

    /**
     * Have to robot turn to a point.
     * 
     * @param angle The angle at which the robot should travel.
     * @param speed The speed at which the robot should travel.
     * @return The speed for the left and right sides of the robot.
     */
    public double[] turn(double angle, double speed) {

        speed = 0;
        double proportionAdjustment = 0;
        double integeralAdjustment = 0;

        if (angle > 180) {
            // ...go the other way around the circle.
            angle -= 360;
        }

        proportionAdjustment = Math.max(0.1525,
                Math.min(1.0, (Math.abs(angle) / 360)));

        speedAdjustment = proportionAdjustment + integeralAdjustment;

        leftThrust = angle > 0 ? Math.abs(speed + speedAdjustment)
                : -Math.abs(speed + speedAdjustment);

        rightThrust = angle > 0 ? -Math.abs(speed + speedAdjustment)
                : Math.abs(speed + speedAdjustment);

        return new double[] { leftThrust, rightThrust };

    }

    /**
     * Calculates the difference between the robot heading and the desired
     * heading.
     * 
     * @param desiredX Final x position.
     * @param desiredY Final y position.
     * @return Heading difference in degrees.
     */
    private double findDesiresdAngle(double desiredX, double desiredY) {

        double desiredHeading = RobotMath.modAngleDegrees(Math.toDegrees(
                Math.atan2(desiredX - pose.getX(), desiredY - pose.getY())));

        double headingDifference = RobotMath
                .modAngleDegrees(desiredHeading - pose.getHeading());

        if (headingDifference > 180) {
            headingDifference -= (360);
        }

        return headingDifference;

    }

}