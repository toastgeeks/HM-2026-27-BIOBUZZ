
package org.firstinspires.ftc.teamcode.subSystems.NewSubSystems;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;

public class Shooter {

    private final Follower follower;
    private final ShooterConstants hardware;

    // Hive coordinates
    private static final double HIVE_X = 58.0;
    private static final double HIVE_LOWER_Y = 55.0;
    private static final double HIVE_UPPER_Y = 84.0;

    // Robot center to turret pivot offset.
    // Set these after measuring your robot.
    private static final double TURRET_OFFSET_X = 0;
    private static final double TURRET_OFFSET_Y = 0;

    public Shooter(Follower follower, ShooterConstants hardware) {
        this.follower = follower;
        this.hardware = hardware;
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double getTargetY(Pose pose) {

        return Math.abs(pose.y() - HIVE_LOWER_Y)
                < Math.abs(pose.y() - HIVE_UPPER_Y)
                ? HIVE_LOWER_Y
                : HIVE_UPPER_Y;
    }

    public ShotSolution calculateShot() {

        Pose pose = follower.pose();

        double vx = follower.velocity().vx;
        double vy = follower.velocity().vy;

        return calculateShot(pose, vx, vy);
    }

    public ShotSolution calculateShot(
            Pose pose,
            double robotVx,
            double robotVy
    ) {

        // ======================================
        // TARGET GEOMETRY
        // ======================================

        double targetX = HIVE_X;
        double targetY = getTargetY(pose);

        double heading = pose.heading();

        // Turret pivot location in field coordinates.
        double pivotX = pose.x()
                + TURRET_OFFSET_X * Math.cos(heading)
                - TURRET_OFFSET_Y * Math.sin(heading);

        double pivotY = pose.y()
                + TURRET_OFFSET_X * Math.sin(heading)
                + TURRET_OFFSET_Y * Math.cos(heading);

        double dx = targetX - pivotX;
        double dy = targetY - pivotY;

        double targetBearing = Math.atan2(dy, dx);

        double rawDistance = Math.hypot(dx, dy);

        double distance = Math.max(
                1,
                rawDistance
                        - ShooterConstants.PASS_THROUGH_POINT_RADIUS
        );

        // ======================================
        // INITIAL BALLISTIC CALCULATION
        // ======================================

        double height = ShooterConstants.SCORE_HEIGHT;

        double scoreAngle = ShooterConstants.SCORE_ANGLE;

        double gravity = ShooterConstants.GRAVITY;

        double hoodAngle = Math.atan(
                (2.0 * height / distance)
                        - Math.tan(scoreAngle)
        );

        hoodAngle = clamp(
                hoodAngle,
                ShooterConstants.HOOD_MIN_ANGLE,
                ShooterConstants.HOOD_MAX_ANGLE
        );

        double denominator =
                2.0 * Math.pow(Math.cos(hoodAngle), 2)
                        * (
                        distance * Math.tan(hoodAngle)
                                - height
                );

        if (denominator <= 0) {
            return new ShotSolution(
                    1620,
                    hoodAngle,
                    0,
                    distance
            );
        }

        double flywheelSpeed = Math.sqrt(
                gravity * distance * distance / denominator
        );

        // ======================================
        // ROBOT VELOCITY COMPENSATION
        // ======================================

        double unitX = dx / rawDistance;
        double unitY = dy / rawDistance;

        // Robot velocity toward/away from target.
        double parallelVelocity =
                robotVx * unitX + robotVy * unitY;

        // Robot velocity perpendicular to target.
        double perpendicularVelocity =
                -robotVx * unitY + robotVy * unitX;

        // Approximate projectile flight time.
        double horizontalSpeed =
                flywheelSpeed * Math.cos(hoodAngle);

        double flightTime = distance / horizontalSpeed;

        // Required horizontal velocity relative to robot.
        double requiredForwardVelocity =
                distance / flightTime - parallelVelocity;

        // Correct the turret for sideways robot motion.
        double turretCompensation = Math.atan2(
                -perpendicularVelocity,
                requiredForwardVelocity
        );

        double compensatedHorizontalSpeed = Math.sqrt(
                requiredForwardVelocity * requiredForwardVelocity
                        + perpendicularVelocity * perpendicularVelocity
        );

        // Preserve the original vertical launch component.
        double verticalSpeed =
                flywheelSpeed * Math.sin(hoodAngle);

        // Recalculate hood and flywheel speed.
        hoodAngle = Math.atan2(
                verticalSpeed,
                compensatedHorizontalSpeed
        );

        hoodAngle = clamp(
                hoodAngle,
                ShooterConstants.HOOD_MIN_ANGLE,
                ShooterConstants.HOOD_MAX_ANGLE
        );

        flywheelSpeed = Math.sqrt(
                compensatedHorizontalSpeed
                        * compensatedHorizontalSpeed
                        + verticalSpeed * verticalSpeed
        );

        // ======================================
        // TURRET AIMING
        // ======================================

        double turretAngle = angleWrap(
                targetBearing
                        - heading
                        + turretCompensation
        );

        turretAngle = clamp(
                turretAngle,
                ShooterConstants.TURRET_MIN_ANGLE,
                ShooterConstants.TURRET_MAX_ANGLE
        );

        // ======================================
        // CONVERT FLYWHEEL SPEED
        // ======================================

        double flywheelTicks =
                94.501 * flywheelSpeed / 12.0
                        - 187.96
                        + ShooterConstants.flywheelOffset;

        flywheelTicks = Math.max(
                ShooterConstants.FLYWHEEL_MIN_SPEED,
                flywheelTicks
        );

        return new ShotSolution(
                flywheelTicks,
                hoodAngle,
                turretAngle,
                distance
        );
    }

    // ======================================
    // APPLY SHOOTING SOLUTION
    // ======================================

    public void updateShooter(ShotSolution solution) {

        hardware.setTargetVelocity(solution.flywheelVelocity);

        hardware.setHoodAngle(solution.hoodAngle);

        hardware.setTurretAngle(solution.turretAngle);
    }

    public ShooterConstants getHardware() {
        return hardware;
    }
}
