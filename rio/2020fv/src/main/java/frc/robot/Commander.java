package frc.robot;

import java.util.Queue;

import disc.data.Instruction;
import disc.data.Scenario;
import disc.data.WaypointMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Commander {

    Scenario scenario;
    WaypointMap waypoints;

    Instruction current;
    String[] currentArgs;
    Queue<Instruction> queue;

    WaypointTravel guidance;

    int counter = 0;

    boolean isDone = true;

    Commander(Scenario scenario, WaypointMap waypoints, WaypointTravel guidance) {
        this.scenario = scenario;
        this.waypoints = waypoints;

        queue = scenario.getInstructionQueue();
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