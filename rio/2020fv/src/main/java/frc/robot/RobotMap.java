package frc.robot;

public class RobotMap {

    // CAN bus IDs for the motor controllers for the drive train
    public static final int MOTOR_FRONT_LEFT = 10;
    public static final int MOTOR_FRONT_RIGHT = 12;
    public static final int MOTOR_REAR_LEFT = 11;
    public static final int MOTOR_REAR_RIGHT = 14;

    // CANID for Manip Controllers
    public static final int INTAKE_CANID = 888;
    public static final int INDEX_BOTTOM_BELT_CANID = 888;
    public static final int INDEX_FRONT_UPPER_BELT_CANID = 888;
    public static final int INDEX_FRONT_LOWER_BELT_CANID = 888;
    public static final int INDEX_BACK_BELT_CANID = 888;
    public static final int SHOOTER_CANID = 13;
    public static final int COLOR_WHEEL_MOTOR_CANID = 888;
    public static final int DS_REVERSE_CHANNEL = 1;
    public static final int DS_FORWARD_CHANNEL = 0;

    // USB IDs in the DS for the controller.
    public static final int LEFT_JOYSTICK = 0;
    public static final int RIGHT_JOYSTICK = 1;
    public static final int GAMEPAD_PORT = 2;

     // Button ID values on the Joysticks
     public static final int JOYSTICK_TRIGGER = 1;
     public static final int JOYSTICK_BOTTOM_BUTTON = 2;
     public static final int JOYSTICK_CENTER_BUTTON = 3;
     public static final int JOYSTICK_LEFT_BUTTON = 4;
     public static final int JOYSTICK_RIGHT_BUTTON = 5;
     public static final int JOYSTICK_BASE_FORWARD_LEFT_BUTTON = 7;
     public static final int JOYSTICK_BASE_FORWARD_RIGHT_BUTTON = 10;
 
     // Axes values for Joysticks
     public static final int JOYSTICK_X_AXIS = 0;
     public static final int JOYSTICK_Y_AXIS = 1;
     public static final int JOYSTICK_Z_AXIS = 2;
 
     // Button values for gamepad
     public static final int A_BUTTON = 1;
     public static final int B_BUTTON = 2;
     public static final int X_BUTTON = 3;
     public static final int Y_BUTTON = 4;
     public static final int GP_L_BUTTON = 5;
     public static final int GP_R_BUTTON = 6;
 
     // Axes values for gamepad
     public static final int GP_L_X_AXIS = 0;
     public static final int GP_L_Y_AXIS = 1;
 
     public static final int GP_L_TRIGGER = 2;
     public static final int GP_R_TRIGGER = 3;
 
     public static final int GP_R_X_AXIS = 4;
     public static final int GP_R_Y_AXIS = 5; 

    // ID for other motor controllers
    public static final int ELEVATOR_MOTOR = 20;
    public static final int BALL_INTAKE_MOTOR = 21;

    // IDs for pneumatic controls
    public static final int PCM = 5;
    public static final int SOLENOID = 1;

    public static final int RECEIVER_SOCKET = 5806;

    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double RAMP_RATE = 0.2;

    public static final double CLICKS_PER_INCH = 0.439373614;
    public static final double WHEEL_BASE = CLICKS_PER_INCH * 21.5;

    public static final double COLOR_WHEEL_SPEED = 0.75;

    public static final double POSITION_TOLERENCE = 4;
    public static final double ANGLE_TOLERENCE = 4;


}
