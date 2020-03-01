/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
    JetsonLight jetsonLight;

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
    int counter = 0;

    @Override
    public void robotInit() {

        try {
            map = new WaypointMap(new File("/home/lvuser/Waypoints2020.txt"));
            autoScenario = new Scenario(new File("/home/lvuser/AutoPlay5.txt"));
            SmartDashboard.putBoolean("Scenario loaded", false);
        }
        catch (FileNotFoundException e) {
            e.printStackTrace();
            SmartDashboard.putBoolean("Scenario loaded", true);
        }

        oi = new OI();
        drive = new DriveTrain();
        intake = new Intake(oi);
        shooter = new Shooter(oi, location, turret, map);
        index = new Indexing(oi, intake, shooter);
        climb = new Climber(oi);
        turret = new Turret(oi);

        imu = new IMU();
        send = new UDPSender();
        receive = new UDPReceiver();
        location = new DeadReckoning(drive, imu, receive, map);
        guidence = new WaypointTravel(drive, location);
        nav = new Navigation(oi, drive, guidence);
        jetsonLight = new JetsonLight(oi);

        receiverThread = new Thread(receive);

        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(1);


        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(1);

        compressor = new Compressor(RobotMap.PCM);

        //SmartDashboard.putData("Auto Scenarios", autoChooser);
        auto = new Commander(autoScenario, map, location, guidence, intake,
                index, shooter);

        receiverThread.start();

    }

    @Override
    public void autonomousInit() {
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
        location.reset();
        send.sendMessage();
    }

    @Override
    public void teleopPeriodic() {
        location.updateTracker();
        location.updateDashboard();
        nav.navTeleopPeriodic();
        climb.climberTeleopPeriodic();
        intake.intakeTeleopPeriodic();
        index.indexManualControls();
        shooter.shooterTeleopPeriodic();
        turret.turretTeleopPeriodic();
        jetsonLight.jetsonLightPeriodic();
        SmartDashboard.putString("pos", receive.getTargetPosition().toString());
    }

    @Override
    public void testPeriodic() {

    }

}