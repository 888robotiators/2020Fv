package frc.robot;

import edu.wpi.first.wpilibj.Solenoid;

public class JetsonLight {
    OI oi;
    Solenoid jetsonLight;

    public JetsonLight(OI oi) {
        jetsonLight = new Solenoid(RobotMap.JETSON_RINGLIGHT_CHANNEL);
        jetsonLight.set(false);
        this.oi = oi;
    }

    public void jetsonLightPeriodic() {
        jetsonLight.set(oi.getRightStickButton(RobotMap.JOYSTICK_3D_TRIGGER));
    }
}
