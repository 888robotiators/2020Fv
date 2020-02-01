/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import disc.data.Waypoint;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Allows the robot to navigate around the field.
 */
public class Navigation {

    OI oi;
    DriveTrain drive;
    WaypointTravel guidance;
    //UDPReceiver receive;

    WaypointMap waypoints;

    Servo servo;

    ControllerButton gamepadButton = ControllerButton.BUTTON_NULL;

    Waypoint destination;

    double[] targetData;

    double autoSpeed;

    boolean reverse = false;
    boolean reverseButton = false;
    boolean reverseButtonLast = false;

    boolean brake = false;
    boolean brakeButton = false;
    boolean brakeButtonLast = false;
    boolean press = false;

    boolean aButton = false;
    boolean bButton = false;
    boolean xButton = false;
    boolean yButton = false;
    boolean aButtonLast = false;
    boolean bButtonLast = false;
    boolean xButtonLast = false;
    boolean yButtonLast = false;

    boolean travel = false;

    /**
     * Constructor
     * 
     * @param p_oi Global {@link OI} object
     * @param p_drive Global {@link DriveTrain} object
     * @param p_guidance Global {@link WaypointTravel} object
     * @param p_receive Global {@link UDPReceiver} object
     */
    public Navigation(OI oi, DriveTrain drive, WaypointTravel guidance, WaypointMap waypoints) { //           UDPReceiver p_receive) {

        this.oi = oi;
        this.drive = drive;
        this.guidance = guidance;
        this.waypoints = waypoints;
        //receive = p_receive;

        servo = new Servo(0);

    }

    /**
     * Method to be run periodically while the robot in enabled.
     */
    public void navTeleopPeriodic() {
        getButton();
        manualDrive();
    }

    public void navAutoPeriodic() {
        autoMove();
    }

    /**
     * Allows for manual control of the drive train.
     */
    public void manualDrive() {

        
        // brake = (brakeButton && !brakeButtonLast) ? !brake : brake;
        // brakeButtonLast = brakeButton;
        // brakeButton = oi.getLeftStickButton(RobotMap.JOYSTICK_LEFT_BUTTON) ||
        //     oi.getRightStickButton(RobotMap.JOYSTICK_RIGHT_BUTTON);
            
        // if (brake) {
        //     drive.brake();
        // }   
        // else { 
        //     drive.coast();
        // }
        
        // SmartDashboard.putBoolean("mode", brake);
        

        // Gets joystick values.
        double leftStickValue = oi.getLeftStickAxis(RobotMap.JOYSTICK_Y_AXIS);
        double rightStickValue = oi.getRightStickAxis(RobotMap.JOYSTICK_Y_AXIS);

        SmartDashboard.putNumber("Left joystick", leftStickValue);
        SmartDashboard.putNumber("Right joystick", rightStickValue);

        // Toggle front of the drive train.
        press = (reverseButton && !reverseButtonLast);

        reverse = press ? !reverse : reverse;

        // Makes camera face the front of the drive train.
        if (press) {
            if (servo.getAngle() == 175) {
                servo.setAngle(10);
            }
            else {
                servo.setAngle(175);
            }
        }

        reverseButtonLast = reverseButton;

        reverseButton = oi.getLeftStickButton(RobotMap.JOYSTICK_BOTTOM_BUTTON)
                || oi.getRightStickButton(RobotMap.JOYSTICK_BOTTOM_BUTTON);

        double leftInput = reverse ? -rightStickValue : leftStickValue;
        double rightInput = reverse ? -leftStickValue : rightStickValue;

        // Sends values to drive train.
        if (Math.abs(leftStickValue) > RobotMap.JOYSTICK_DEADZONE || 
                Math.abs(rightStickValue) > RobotMap.JOYSTICK_DEADZONE) {

            leftInput = (Math.abs(leftInput) < RobotMap.JOYSTICK_DEADZONE) ? 0.0 : leftInput;
            rightInput = (Math.abs(rightInput) < RobotMap.JOYSTICK_DEADZONE) ? 0.0 : rightInput;

            leftInput = (Math.abs(leftInput) - RobotMap.JOYSTICK_DEADZONE) 
                    * (1 / (1 - RobotMap.JOYSTICK_DEADZONE)) * Math.signum(leftInput);
            rightInput = (Math.abs(rightInput) - RobotMap.JOYSTICK_DEADZONE) 
                    * (1 / (1 - RobotMap.JOYSTICK_DEADZONE)) * Math.signum(rightInput);

            if (!oi.getLeftStickButton(RobotMap.JOYSTICK_TRIGGER)) {
                leftInput *= 0.7;
                rightInput *= 0.7;
            }

            drive.move(leftInput, rightInput);

        }

        else {

            drive.move(0, 0);

        }

    }

    public void autoMove() {
        if (travel) {
            if (guidance.goToWaypoint(destination, autoSpeed)) {
                travel = false;
            }
        }
    }

    public void goTo(String waypointName, double speed) {
        destination = waypoints.get(waypointName);
        autoSpeed = speed;
        travel = true;
    }

    /**
     * Tells if the robot is in reverse.
     * 
     * @return True if the robot in in reverse.
     */
    public boolean isReverse() {
        return reverse;
    }

    /**
     * Updates an an enum that holds the last command from the gamepad letter
     * buttons.
     */
    public void getButton() {
        aButtonLast = aButton;
        bButtonLast = bButton;
        xButtonLast = xButton;
        yButtonLast = yButton;

        aButton = oi.getGamepadButton(RobotMap.A_BUTTON);
        bButton = oi.getGamepadButton(RobotMap.B_BUTTON);
        xButton = oi.getGamepadButton(RobotMap.X_BUTTON);
        yButton = oi.getGamepadButton(RobotMap.Y_BUTTON);

        if (aButton) {
            gamepadButton = ControllerButton.BUTTON_A;
        }
        else if (bButton) {
            gamepadButton = ControllerButton.BUTTON_B;
        }
        else if (xButton) {
            gamepadButton = ControllerButton.BUTTON_X;
        }
        else if (yButton) {
            gamepadButton = ControllerButton.BUTTON_Y;
        }
    }

    public enum ControllerButton {
        BUTTON_A, BUTTON_B, BUTTON_X, BUTTON_Y, BUTTON_NULL;

    }
}
