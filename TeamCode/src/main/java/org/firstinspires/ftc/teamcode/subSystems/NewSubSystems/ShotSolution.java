
package org.firstinspires.ftc.teamcode.subSystems.NewSubSystems;

public class ShotSolution {

    public final double flywheelVelocity;
    public final double hoodAngle;
    public final double turretAngle;
    public final double distance;

    public ShotSolution(
            double flywheelVelocity,
            double hoodAngle,
            double turretAngle,
            double distance
    ) {
        this.flywheelVelocity = flywheelVelocity;
        this.hoodAngle = hoodAngle;
        this.turretAngle = turretAngle;
        this.distance = distance;
    }
}
