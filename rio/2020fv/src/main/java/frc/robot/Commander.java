package frc.robot;

import java.util.Queue;

import disc.data.Instruction;
import disc.data.Scenario;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Commander {

    private Scenario scenario;
    private  WaypointMap waypoints;

    private Instruction current;
    private String[] currentArgs;
    private Queue<Instruction> queue;

    private WaypointTravel guidance;
    //private DeadReckoning location;
    private Intake intake;
    //private Indexing index;
    //private Shooter shooter;

    private int counter = 0;

    private boolean isDone = true;

    Commander(Scenario scenario, WaypointMap waypoints, DeadReckoning location, 
            WaypointTravel guidance, Intake intake, Indexing index, Shooter shooter) {
        this.scenario = scenario;
        this.waypoints = waypoints;

        queue = this.scenario.getInstructionQueue();
        current = null;

        this.guidance = guidance;
    }

    public void periodic() {
        if (isDone) {
            isDone = false;
            if (!queue.isEmpty()) {
                current = queue.remove();
                counter++;
            }
            else {
                current = null;
            }
        }

        SmartDashboard.putNumber("instruction", counter);

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
                        if (guidance.goToWaypoint(waypoints.get(currentArgs[1]), Double.parseDouble(currentArgs[2]))) {
                            isDone = true;
                        }
                    }

                    break;

                case "shooter":
                    break;
                case "intake":
                        if (currentArgs[0].equalsIgnoreCase("intakeDown")) {
                            intake.flipDown();
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
                        if (currentArgs[0].equalsIgnoreCase("intakeRun")) {
                            intake.intakeIn();
                            isDone = true;
                        }
                        if (currentArgs[0].equalsIgnoreCase("intakeStop")) {
                            intake.intakeStop();
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