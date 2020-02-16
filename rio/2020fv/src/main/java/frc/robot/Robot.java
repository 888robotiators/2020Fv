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
    UDPReceiver receive;
    UDPSender sender;
    UsbCamera camera0;
    UsbCamera camera1;
    JetsonLight jetsonLight;
    

    // Pneumatics
    Compressor compressor;

    @Override
    public void robotInit() {
        receive = new UDPReceiver();
        sender = new UDPSender();
        oi = new OI();
        drive = new DriveTrain();
        nav = new Navigation(oi, drive);//, guidence);
        intake = new Intake(oi);
        index = new Indexing(oi, intake);
        climb = new Climber(oi);
        turret = new Turret(oi);
        jetsonLight = new JetsonLight(oi);
        shooter = new  Shooter(oi, index, receive, turret);

        compressor = new Compressor(RobotMap.PCM);

        camera0 = CameraServer.getInstance().startAutomaticCapture(0);
        camera1 = CameraServer.getInstance().startAutomaticCapture(1);
    
        // robot kermits sause
    }

    @Override
    public void autonomousInit() {
        //location.reset();
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        compressor.setClosedLoopControl(true);
        //sender.sendMessage();
    }

    @Override
    public void teleopPeriodic() {
        nav.navTeleopPeriodic();
        climb.climberTeleopPeriodic();
        intake.intakeTeleopPeriodic();
        index.indexPeriodic();
        shooter.shooterTeleopPeriodic();
        //jetsonLight.jetsonLightPeriodic();
        turret.turretTeleopPeriodic();
        //receive.run();
    }

    @Override
    public void testPeriodic() {

    }
}