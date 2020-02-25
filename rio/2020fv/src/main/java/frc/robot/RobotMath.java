package frc.robot;

import disc.data.Waypoint;

public class RobotMath {

    public static double modAngleRadians(double angle) {
        if (angle < 0) {
            angle += 2 * Math.PI;
        }
        angle %= 2 * Math.PI;
        return angle;
    }

    public static double modAngleDegrees(double angle) {
        if (angle < 0) {
            angle += 720;
        }
        angle %= 360;
        return angle;
    }

    public static Waypoint translatePoint(Waypoint point,
            Waypoint translation) {
        return new Waypoint(point.getX() + translation.getX(),
                point.getY() + translation.getY());
    }
}