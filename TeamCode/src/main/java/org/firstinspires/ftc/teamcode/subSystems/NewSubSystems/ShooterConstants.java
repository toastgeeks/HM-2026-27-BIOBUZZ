
package org.firstinspires.ftc.teamcode.subSystems.NewSubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class ShooterConstants {


    // Actual Shooter Constants

    public static final double SCORE_HEIGHT = 59.0;

    // Desired angle of the ball as it reaches the target.
    // Negative means descending.
    public static final double SCORE_ANGLE = Math.toRadians(-70);

    public static final double PASS_THROUGH_POINT_RADIUS = 3.0;

    public static final double GRAVITY = 12.174 * 12.0;

    public static final double HOOD_MAX_ANGLE = Math.toRadians(89);
    public static final double HOOD_MIN_ANGLE = Math.toRadians(45);

    private double lastHoodAngle = Double.NaN;
    private double lastTurretAngle = Double.NaN;

    private static final double HOOD_UPDATE_THRESHOLD =
            Math.toRadians(0.5);

    private static final double TURRET_UPDATE_THRESHOLD =
            Math.toRadians(1.0);

    // flywheel stuff

    public static double flywheelOffset = 0;

    public static double FLYWHEEL_MIN_SPEED = 200;

    public static final double SHOOTER_P = 266.8137;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 13.0907;

    public static final double VELOCITY_STEP = 10;

    private double targetVelocity = 1620;
    private boolean shooterOn = false;

    private static final double READY_TOLERANCE = 50;
    private static final long READY_STABLE_TIME = 150;

    private long readySinceNanos = -1;

    // gate and transfer stuff

    public static final double GATE_OPEN_POSITION = 0.3;
    public static final double GATE_CLOSED_POSITION = 0.5;

    private double gatePosition = GATE_CLOSED_POSITION;

    // ==========================================
    // HOOD CALIBRATION
    // ==========================================

    // These are initial servo position estimates.
    // Calibrate on the actual robot.

    public static double HOOD_SERVO_MIN_POSITION = 0.27;
    public static double HOOD_SERVO_MAX_POSITION = 0.99;

    // ==========================================
    // TURRET CALIBRATION
    // ==========================================

    // Turret angle limits relative to the robot.
    public static double TURRET_MIN_ANGLE = Math.toRadians(-90);
    public static double TURRET_MAX_ANGLE = Math.toRadians(90);

    public static double TURRET_MIN_POSITION = 0.05;
    public static double TURRET_MAX_POSITION = 0.95;

    // Reverse either servo if the mounting is mirrored.
    public static boolean TURRET_SERVO_1_REVERSED = false;
    public static boolean TURRET_SERVO_2_REVERSED = false;

    // ==========================================
    // SERVO SETTLING
    // ==========================================

    public static long SERVO_SETTLE_TIME_MS = 300;

    private long hoodCommandTime = 0;
    private long turretCommandTime = 0;

    // ==========================================
    // HARDWARE
    // ==========================================

    private final DcMotorEx shooter1;
    private final DcMotorEx shooter2;
    private final DcMotorEx transfer;

    private final Servo gate;
    private final Servo hood;
    private final Servo turret1;
    private final Servo turret2;

    private double hoodPosition = HOOD_SERVO_MIN_POSITION;
    private double turretAngle = 0;

    private boolean firing = false;

    public ShooterConstants(HardwareMap hw) {

        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter2 = hw.get(DcMotorEx.class, "shooter2");
        transfer = hw.get(DcMotorEx.class, "transfer");

        gate = hw.get(Servo.class, "gate");
        hood = hw.get(Servo.class, "hood");

        turret1 = hw.get(Servo.class, "turret1");
        turret2 = hw.get(Servo.class, "turret2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);

        transfer.setPower(0);

        setGatePosition(GATE_CLOSED_POSITION);

        setShooterPIDF(
                SHOOTER_P,
                SHOOTER_I,
                SHOOTER_D,
                SHOOTER_F
        );

        setHoodAngle(HOOD_MIN_ANGLE);
        setTurretAngle(0);
    }

    // ==========================================
    // SERVO CONVERSION
    // ==========================================

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    private double hoodAngleToPosition(double angle) {

        double fraction =
                (angle - HOOD_MIN_ANGLE)
                        / (HOOD_MAX_ANGLE - HOOD_MIN_ANGLE);

        return HOOD_SERVO_MIN_POSITION
                + fraction * (
                HOOD_SERVO_MAX_POSITION
                        - HOOD_SERVO_MIN_POSITION
        );
    }

    private double turretAngleToPosition(double angle) {

        double fraction =
                (angle - TURRET_MIN_ANGLE)
                        / (TURRET_MAX_ANGLE - TURRET_MIN_ANGLE);

        return TURRET_MIN_POSITION
                + fraction * (
                TURRET_MAX_POSITION
                        - TURRET_MIN_POSITION
        );
    }

    public void setHoodAngle(double angle) {

        angle = clamp(
                angle,
                HOOD_MIN_ANGLE,
                HOOD_MAX_ANGLE
        );

        if (!Double.isNaN(lastHoodAngle)
                && Math.abs(angle - lastHoodAngle)
                < HOOD_UPDATE_THRESHOLD) {
            return;
        }

        lastHoodAngle = angle;

        hoodPosition = clamp(
                hoodAngleToPosition(angle),
                0,
                1
        );

        hood.setPosition(hoodPosition);

        hoodCommandTime = System.currentTimeMillis();
    }

    public void setTurretAngle(double angle) {

        angle = clamp(
                angle,
                TURRET_MIN_ANGLE,
                TURRET_MAX_ANGLE
        );

        if (!Double.isNaN(lastTurretAngle)
                && Math.abs(angle - lastTurretAngle)
                < TURRET_UPDATE_THRESHOLD) {
            return;
        }

        lastTurretAngle = angle;

        turretAngle = angle;

        double position = clamp(
                turretAngleToPosition(angle),
                TURRET_MIN_POSITION,
                TURRET_MAX_POSITION
        );

        double position1 = TURRET_SERVO_1_REVERSED
                ? 1.0 - position : position;

        double position2 = TURRET_SERVO_2_REVERSED
                ? 1.0 - position : position;

        turret1.setPosition(position1);
        turret2.setPosition(position2);

        turretCommandTime = System.currentTimeMillis();
    }

    public boolean areServosSettled() {

        long now = System.currentTimeMillis();

        return now - hoodCommandTime >= SERVO_SETTLE_TIME_MS
                && now - turretCommandTime >= SERVO_SETTLE_TIME_MS;
    }

    public double getHoodPosition() {
        return hoodPosition;
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getTurretPosition() {
        return turretAngleToPosition(turretAngle);
    }

    // ==========================================
    // FLYWHEEL
    // ==========================================

    public void setShooterPIDF(double p, double i, double d, double f) {

        for (DcMotorEx motor :
                new DcMotorEx[]{shooter1, shooter2}) {

            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            motor.setPIDFCoefficients(
                    DcMotor.RunMode.RUN_USING_ENCODER,
                    new PIDFCoefficients(p, i, d, f)
            );
        }
    }

    public PIDFCoefficients getShooterPIDF() {
        return shooter1.getPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER
        );
    }

    public void setShooterVelocity(double velocity) {
        shooter1.setVelocity(velocity);
        shooter2.setVelocity(velocity);
    }

    public double getShooterVelocity() {
        return Math.abs(shooter1.getVelocity());
    }

    public double getShooter2Velocity() {
        return Math.abs(shooter2.getVelocity());
    }

    public double getShooterCurrentAmps() {
        return shooter1.getCurrent(CurrentUnit.AMPS);
    }

    public double getShooter2CurrentAmps() {
        return shooter2.getCurrent(CurrentUnit.AMPS);
    }

    public void setTargetVelocity(double velocity) {

        if (Math.abs(targetVelocity - velocity) > 0.5) {

            targetVelocity = velocity;
            readySinceNanos = -1;

            if (shooterOn) {
                setShooterVelocity(targetVelocity);
            }
        }
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void increaseTargetVelocity() {
        setTargetVelocity(targetVelocity + VELOCITY_STEP);
    }

    public void decreaseTargetVelocity() {
        setTargetVelocity(Math.max(0, targetVelocity - VELOCITY_STEP));
    }

    private void spinUp() {
        shooterOn = true;
        setShooterVelocity(targetVelocity);
    }

    private void spinDown() {
        shooterOn = false;
        setShooterVelocity(0);
        readySinceNanos = -1;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public boolean isShooterReady() {

        if (!shooterOn || targetVelocity <= 0) {
            readySinceNanos = -1;
            return false;
        }

        boolean ready1 =
                Math.abs(targetVelocity - getShooterVelocity())
                        <= READY_TOLERANCE;

        boolean ready2 =
                Math.abs(targetVelocity - getShooter2Velocity())
                        <= READY_TOLERANCE;

        if (ready1 && ready2) {

            if (readySinceNanos == -1) {
                readySinceNanos = System.nanoTime();
            }

            return System.nanoTime() - readySinceNanos
                    >= READY_STABLE_TIME * 1_000_000;
        }

        readySinceNanos = -1;
        return false;
    }

    // ==========================================
    // IVY COMMANDS
    // ==========================================

    public Command rev() {
        return instant(() -> spinUp()).requiring(this);
    }

    public Command stopRev() {
        return instant(() -> spinDown()).requiring(this);
    }

    public Command stop() {
        return stopRev();
    }

    public boolean isFiring() {
        return firing;
    }

    public Command fire(double transferTime) {

        return sequential(
                instant(() -> firing = true),
                openGate(),
                waitMs(200),
                instant(() -> transfer.setPower(1.0)),
                waitMs(transferTime),
                instant(() -> transfer.setPower(0)),
                closeGate(),
                instant(() -> firing = false),
                stopRev()
        ).requiring(this);
    }

    public Command transfer(double power) {
        return instant(
                () -> transfer.setPower(power)
        ).requiring(this);
    }

    public Command transferStop() {
        return instant(
                () -> transfer.setPower(0)
        ).requiring(this);
    }

    // ==========================================
    // GATE
    // ==========================================

    private void setGatePosition(double position) {
        gatePosition = position;
        gate.setPosition(position);
    }

    public Command openGate() {
        return instant(
                () -> setGatePosition(GATE_OPEN_POSITION)
        ).requiring(this);
    }

    public Command closeGate() {
        return instant(
                () -> setGatePosition(GATE_CLOSED_POSITION)
        ).requiring(this);
    }

    public void newServoPose(double position) {
        setGatePosition(position);
    }

    public double getServoPose() {
        return gatePosition;
    }
}
