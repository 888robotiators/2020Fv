package frc.robot;

import edu.wpi.first.wpilibj.Relay;

public class JetsonLight {
    protected Relay jetsonLight;
    protected OI oi;

    public JetsonLight(OI oi) {
        jetsonLight = new Relay(RobotMap.JETSON_RINGLIGHT_CHANNEL, Relay.Direction.kForward);

        jetsonLight.set(Relay.Value.kOff);

        this.oi = oi;
    }

    public void jetsonLightPeriodic() {
        if(oi.getRightStickButton(12)) {
            if(jetsonLight.get().equals(Relay.Value.kOff)) {
                jetsonLight.set(Relay.Value.kOn);
            } else {
                jetsonLight.set(Relay.Value.kOff);
            }
        }
    }
}
