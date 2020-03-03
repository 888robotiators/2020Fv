package frc.robot;

import java.util.Queue;

import disc.data.Instruction;
import disc.data.Scenario;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

    private int counter = 0;
    private int instructionCounter = 0;

    private boolean isDone = true;

    Commander(Scenario scenario, WaypointMap waypoints, DeadReckoning location,
            WaypointTravel guidance, Intake intake, Indexing index,
            Shooter shooter) {
        this.scenario = scenario;
        this.waypoints = waypoints;

        queue = this.scenario.getInstructionQueue();
        current = null;

        this.guidance = guidance;
        this.intake = intake;
        this.index = index;
        this.shooter = shooter;
    }

    public void periodic() {
        if (isDone) {
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

        if (current != null) {

            currentArgs = current.getArgs();

            switch (current.getT()) {

                case DELIMITER:

                    if (currentArgs[0].equalsIgnoreCase("start")) {
                        // start up code for scenario
                    }

                    if (currentArgs[0].equalsIgnoreCase("stop")) {
                        // stop code for scenario
                    }

                    break;

                case COMMAND:

                    switch (current.getTarget()) {
                        case "nav":

                            if (currentArgs[0].equalsIgnoreCase("goTo")) {
                                if (guidance.goToWaypoint(
                                        waypoints.get(currentArgs[1]),
                                        Double.parseDouble(currentArgs[2]))) {
                                    isDone = true;
                                }
                            }

                            break;

                        case "shooter":
                            if (currentArgs[0].equalsIgnoreCase("shoot")) {
                                if (index.hasBalls()) {
                                    index.bringToTop();
                                    shooter.setShooterOutputVelocity(Integer.parseInt(currentArgs[1]));
                                    if (shooter.readyToFire()) {
                                        index.loadShooter();
                                    }
                                }
                                else {
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
                        case "intake":
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
                        case "command":
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
                        default:
                            isDone = true;
                            break;
                    }

                    break;

                case CONTROL_STATE:
                    isDone = true;
                    break;
            }

        }

    }

}