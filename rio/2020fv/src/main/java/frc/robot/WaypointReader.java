package frc.robot;

import java.io.IOException;

import disc.data.WaypointMap;

public class WaypointReader {

    WaypointMap waypoints;

    public WaypointReader(String fileName) throws IOException {
        
        waypoints = new WaypointMap("/home/lvuser/" + fileName + ".txt");

    }

    public WaypointMap getWaypoints() {
        return waypoints;
    }

}