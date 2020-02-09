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

import edu.wpi.first.wpilibj.Compressor;
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
    Commander auto;

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
        nav = new Navigation(oi, drive, guidence);

        compressor = new Compressor(RobotMap.PCM);

        auto = new Commander(autoScenario, waypoints, guidence);

        // robot kermits sause

    }

    @Override
    public void autonomousInit() {
        location.reset();
    }

    @Override
    public void autonomousPeriodic() {
        location.updateTracker();
        location.updateDashboard();
        auto.periodic();
    }

    @Override
    public void teleopInit() {
        compressor.setClosedLoopControl(true);
    }

    @Override
    public void teleopPeriodic() {
        //receive.receiveMessage();
        location.updateTracker();
        location.updateDashboard();
        nav.navTeleopPeriodic();
    }

    @Override
    public void testPeriodic() {

    }

}