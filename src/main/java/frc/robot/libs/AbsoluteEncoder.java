package frc.robot.libs;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.RobotBase;

public class AbsoluteEncoder extends AnalogEncoder {

    private final double zeroOffset;

    private double simulatedAngle = 0;

    public AbsoluteEncoder(int analogID, double zeroOffset) {
        // Initializes an encoder on a DIO port and uses 360 as full range and 180 as
        // half of the range
        super(analogID, 360.0, 0.0);

        this.zeroOffset = zeroOffset;
    }

    public AbsoluteEncoder(int analogID, double zeroOffset, double maxAngle) {
        // Initializes an encoder on a DIO port and uses 360 as full range and 180 as
        // half of the range
        super(analogID, maxAngle, 0.0);

        this.zeroOffset = zeroOffset;
    }

    public double getAbsolutePosition() {
        if (!RobotBase.isSimulation()) {
            double position = (super.get() - zeroOffset) % 360.0;
            return position < 0 ? position + 360.0 : position;
        } else {
            return simulatedAngle;
        }
    }

    public void setDistance(double angle) {
        simulatedAngle = angle;
    }
}