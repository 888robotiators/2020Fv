/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.io.File;
import java.io.FileNotFoundException;

import disc.data.Scenario;
import disc.data.WaypointMap;
import disc.tools.Directory;
import disc.tools.Interpreter;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
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
    OI oi;
    DriveTrain drive;
    DeadReckoning location;
    Navigation nav;
    IMU imu;
    WaypointTravel guidence;
    UDPSender send;
    UDPReceiver receive;
    WaypointMap waypoints;

    Scenario autoScenario;
    Directory autoFeatures;
    Interpreter auto;

    // Pneumatics
    Compressor compressor;

    @Override
    public void robotInit() {
        
        try {
            waypoints = new WaypointMap(new File("/home/lvuser/Waypoints2020.txt"));
            autoScenario = new Scenario(new File("/home/lvuser/TestAuto.txt"));
            SmartDashboard.putBoolean("Suicide", false);
        } 
        catch (FileNotFoundException e) {
            e.printStackTrace();
            SmartDashboard.putBoolean("Suicide", true);
        }
        
        oi = new OI();
        drive = new DriveTrain();
        imu = new IMU();
        location = new DeadReckoning(drive, imu);
        guidence = new WaypointTravel(drive, location);
        send = new UDPSender();
        receive = new UDPReceiver();
        nav = new Navigation(oi, drive, guidence, waypoints);

        compressor = new Compressor(RobotMap.PCM);

        autoFeatures = new Directory(nav);
        auto = new Interpreter(autoFeatures, autoScenario);

    }

    @Override
    public void autonomousInit() {
        auto.start();
    }

    @Override
    public void autonomousPeriodic() {
        //receive.receiveMessage();
        location.updateTracker();
        nav.navAutoPeriodic();
    }

    @Override
    public void teleopInit() {
        auto.interrupt();
        compressor.setClosedLoopControl(true);
    }

    @Override
    public void teleopPeriodic() {
        //receive.receiveMessage();
        location.updateTracker();
        nav.navTeleopPeriodic();
    }

    @Override
    public void testPeriodic() {

    }

}