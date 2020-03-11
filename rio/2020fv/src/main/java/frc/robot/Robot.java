/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import java.io.File;
import java.io.FileNotFoundException;

import disc.data.Scenario;
import disc.data.WaypointMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.properties file in the
 * project.
 */
public class Robot extends TimedRobot {
    /* Here are the instantiations of everything used in the class */
    Climber climb;
    DriveTrain drive;
    Indexing index;
    Intake intake;
    Navigation nav;
    OI oi;
    Shooter shooter;
    Turret turret;

    DeadReckoning location;

    IMU imu;
    WaypointTravel guidence;
    UDPSender send;
    UDPReceiver receive;
    WaypointMap map;

    Thread receiverThread;

    UsbCamera camera0;
    UsbCamera camera1;

    Scenario autoScenario;
    Commander auto;

    // Pneumatics
    Compressor compressor;

    boolean done = false;

    int delay;
    int counter ;

    int teleopCounter;

    boolean timeToMove = false;

    String joshAutoSucks = "/home/lvuser/JoshAutoSucks.txt";
    String charlieAutoGood = "/home/lvuser/SixBallAuto.txt";
    String pushyAuto = "/home/lvuser/SixBallPushyBoi.txt";
    String domCode = "/home/lvuser/DomSomething.txt";

    // Changes made 3.9.20
    // Climber: 
        //Climber will start unlocked. Motors cannot raise or lower climber if locked.
        //Put boolean on dashboard if climber is unlocked or locked (true if locked)
        //Added code in teleopPeriodic() (Robot class) that prevents climber operations until last 30 seconds
        //Unlikely we will use this timeout, so it is currently commented out
    //Autonomous:
        //Added a new command moveStraight (case sensitive) with arguments direction (forward or back),
        //distance and speed, in that order (currentArg[0] is direction, must be "forward" or "back", 
        //[1] is distance in inches, [2] is speed in %output). 
        //This command uses clicks per inch only with no navx or waypoints.
        //This command is added to the switch statement in Command class
        //Uploaded text file that uses this command to the rio under the path above
    //Turret:
        //Automatically sets the turret to straight on (0 degrees) when teleop begins as per request by drive team
        //Disabled use of (commented out) turret in teleop as per request by the drive team. 
        //Will likely add back teleop turret usage later

    @Override
    public void robotInit() {

        try {
            map = new WaypointMap(new File("/home/lvuser/Waypoints2020.txt"));
            autoScenario = new Scenario(new File(pushyAuto));
            SmartDashboard.putBoolean("File not found", false);
        }
        catch (FileNotFoundException e) {
            e.printStackTrace();
            SmartDashboard.putBoolean("File not found", true);
        } 

        receive = new UDPReceiver();
        oi = new OI();
        drive = new DriveTrain();
        intake = new Intake(oi);
        shooter = new Shooter(oi, location, turret, map, receive);
        index = new Indexing(oi, intake, shooter);
        turret = new Turret(oi);
        climb = new Climber(oi, turret);

        imu = new IMU();
        send = new UDPSender();
        location = new DeadReckoning(drive, imu, receive, map);
        guidence = new WaypointTravel(drive, location);
        nav = new Navigation(oi, drive, guidence);

        receiverThread = new Thread(receive);

        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(1);

        compressor = new Compressor(RobotMap.PCM);

        auto = new Commander(autoScenario, map, location, guidence, intake,
                index, shooter, turret, drive);

        //receiverThread.start();
        teleopCounter = 0;
    }

    @Override
    public void autonomousInit() {
        drive.resetEncoderOffset();
        compressor.setClosedLoopControl(true);
        location.reset();
    }

    @Override
    public void autonomousPeriodic() {
        location.updateTracker();
        location.updateDashboard();
        index.updateBallPoitions();
        auto.periodic();
    }

    @Override
    public void teleopInit() {
        compressor.setClosedLoopControl(true);
        send.sendMessage();
        drive.coast();
        index.stopLoadShooter();
        location.reset();
        turret.setAngle(0);
        climb.unlock();
    }

    @Override
    public void teleopPeriodic() {
        location.updateTracker();
        location.updateDashboard();
        nav.navTeleopPeriodic();
        //if(teleopCounter > 135 * 50) {
            climb.climberTeleopPeriodic();
        //}
        intake.intakeTeleopPeriodic();
        index.indexManualControls();
        shooter.shooterTeleopPeriodic();
        turret.turretTeleopPeriodic();

        //teleopCounter++;
    }

    @Override
    public void testPeriodic() {

    }

}