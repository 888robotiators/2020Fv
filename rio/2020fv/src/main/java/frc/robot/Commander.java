package frc.robot;

import java.util.Queue;

import disc.data.Instruction;
import disc.data.Scenario;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//
public class Commander {

    private Scenario scenario;
    private WaypointMap waypoints;

    private Instruction current;
    private String[] currentArgs;
    private Queue<Instruction> queue;

    private WaypointTravel guidance;
    private Intake intake;
    private Indexing index;
    private Shooter shooter;
    private Turret turret;
    private DriveTrain drive;

    private int counter = 0;
    private int instructionCounter = 0;

    private boolean indexRun = false;

    private boolean isDone = true;

    Commander(Scenario scenario, WaypointMap waypoints, DeadReckoning location,
            WaypointTravel guidance, Intake intake, Indexing index,
            Shooter shooter, Turret turret, DriveTrain drive) {
        this.scenario = scenario;
        this.waypoints = waypoints;

        queue = this.scenario.getInstructionQueue();
        current = null;

        this.guidance = guidance;
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
        this.turret = turret;
        this.drive = drive;
    }

    public void periodic() {
        if (isDone) { //remove the first item in the queue if the action is done
            isDone = false;
            if (!queue.isEmpty()) {
                current = queue.remove();
                instructionCounter++;
            }
            else {
                current = null;
            }
        }

        SmartDashboard.putNumber("instruction", instructionCounter);
        SmartDashboard.putBoolean("Done auto", current == null);

        if (current != null) {

            currentArgs = current.getArgs();

            switch (current.getT()) {

                case DELIMITER://start and stop the code (not used) 

                    if (currentArgs[0].equalsIgnoreCase("start")) {
                        // start up code for scenario
                    }

                    if (currentArgs[0].equalsIgnoreCase("stop")) {
                        // stop code for scenario
                    }

                    break;

                case COMMAND: 

                    switch (current.getTarget()) {
                        case "nav": //waypoint travel to inputted waypoint at speed 

                            if (currentArgs[0].equalsIgnoreCase("goTo")) {
                                if (guidance.goToWaypoint(
                                        waypoints.get(currentArgs[1]),
                                        Double.parseDouble(currentArgs[2]))) {
                                    isDone = true;
                                }
                            }

                            break;

                        case "shooter": //shooter stuff: shoot - shoots balls, rpms, timeout  
                                        //               spin - spins the shooter, rpms
                            if (currentArgs[0].equalsIgnoreCase("shoot")) {
                                if (index.hasBalls() && counter < Double.parseDouble(currentArgs[2]) * 50) {
                                    counter++;
                                    index.bringToTop();
                                    shooter.setShooterOutputVelocity(Double.parseDouble(currentArgs[1]));
                                    if (shooter.readyToFire()) {
                                        index.loadShooter();
                                    }
                                    else {
                                        index.stopLoadShooter();
                                    }
                                }
                                else {
                                    counter = 0;
                                    shooter.stop();
                                    index.stopIndexer();
                                    isDone = true;
                                }  
                            }
                            if (currentArgs[0].equalsIgnoreCase("spin")) {
                                shooter.setShooterOutputVelocity(Integer.parseInt(currentArgs[1]));
                                isDone = true;
                            }
                            break;
                        case "intake": //Controll intake position and speed
                            if (currentArgs[0].equalsIgnoreCase("intakeDown")) {
                                intake.flipIntake(true);
                                isDone = true;
                            }
                            if (currentArgs[0].equalsIgnoreCase("intakeUp")) {
                                intake.flipUp();
                                isDone = true;
                            }
                            if (currentArgs[0].equalsIgnoreCase("intakeRun")) {
                                intake.intakeIn();
                                isDone = true;
                            }
                            if (currentArgs[0].equalsIgnoreCase("intakeStop")) {
                                intake.intakeStop();
                                isDone = true;
                            }
                            break;
                        case "turret": //tells the turret to rotate to inputted degree 
                            if (currentArgs[0].equalsIgnoreCase("rotateTo")) {
                                if (turret.setAngle(Double.parseDouble(currentArgs[1]))) {
                                    isDone = true;
                                }
                            }
                            break;
                        case "index": //full index on no smarts 
                            if (currentArgs[0].equalsIgnoreCase("start")) {
                                indexRun = true;
                                isDone = true;
                            }
                            if (currentArgs[0].equalsIgnoreCase("stop")) {
                                indexRun = false;
                                isDone = true;
                            }
                            break;
                        case "command": //waits inputted seconds
                            if (currentArgs[0].equalsIgnoreCase("wait")) {
                                if (counter < Integer.parseInt(currentArgs[1]) * 50) {
                                    counter++;
                                }
                                else {
                                    counter = 0;
                                    isDone = true;
                                }
                            }

                            if (currentArgs[0].equalsIgnoreCase("runPlay")) {
                                isDone = true;
                            }
    
                            break;
                        case "moveStraight": //Clicks per inch move straight forward or backwards inputted speed and distance
                                            //distance (inch), speed (%output)
                            if(currentArgs[0].equalsIgnoreCase("back")) {
                                if (drive.getEncoders()[1] > -RobotMap.CLICKS_PER_INCH * Double.parseDouble(currentArgs[1])) {
                                        drive.move(Double.parseDouble(currentArgs[2]), Double.parseDouble(currentArgs[2]));
                                    }
                                else { 
                                    drive.move(0,0);
                                    drive.resetEncoderOffset();
                                    isDone = true;
                                }
                            }
                            else if(currentArgs[0].equals("forward")) {
                                if (drive.getEncoders()[1] < RobotMap.CLICKS_PER_INCH * Double.parseDouble(currentArgs[1])) {
                                    drive.move(-Double.parseDouble(currentArgs[2]), -Double.parseDouble(currentArgs[2]));
                                }
                                else { 
                                    drive.move(0,0);
                                    drive.resetEncoderOffset();
                                    isDone = true;
                                }
                            }

                            break;
                        case "timeMove":
                            //direction, time, sPEED
                            if(currentArgs[0].equalsIgnoreCase("back")) {
                                if (counter < Double.parseDouble(currentArgs[1]) * 50) {
                                    drive.move(Double.parseDouble(currentArgs[2]),Double.parseDouble(currentArgs[2]));
                                    counter++;
                                }
                                else {
                                    drive.move(0,0);
                                    counter = 0;
                                    isDone = true;
                                }
                            }
                            else if(currentArgs[0].equals("forward")) {
                                if (counter < Double.parseDouble(currentArgs[1]) * 50) {
                                    drive.move(-Double.parseDouble(currentArgs[2]),-Double.parseDouble(currentArgs[2]));
                                    counter++;
                                }
                                else {
                                    drive.move(0,0);
                                    counter = 0;
                                    isDone = true;
                                }
                            }
                            
                            break;                            
                        default:
                            isDone = true;
                            break;
                    }

                    break;

                case CONTROL_STATE:
                    isDone = true;
                    break;
            }

            if (indexRun) {
                index.bringToTop();
            }

            SmartDashboard.putNumber("Encoder RPMs", shooter.getRPMs());

        }

    }

}