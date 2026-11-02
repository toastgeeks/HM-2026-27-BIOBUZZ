package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class NEW_SHOOTER_TRANSFER {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx transfer;
    private Servo gate;
    private double servoPose = 0.5;

    public static final double SHOOTER_P = 266.8137;
    public static final double SHOOTER_I = 0;
    public static final double SHOOTER_D = 0;
    public static final double SHOOTER_F = 13.0907;
    public static final double VELOCITY_STEP = 10;

    // Target flywheel velocity in ticks per second
    public double targetVelocity = 1620;
    private boolean shooterOn = false;
    private boolean firing = false;

    // Maximum acceptable difference from target velocity
    private static final double READY_TOLERANCE = 50;

    // How long the shooter must stay within tolerance
    private static final long READY_STABLE_TIME = 150;

    // Tracks how long both flywheels have been ready
    private long readySince = -1;

    public boolean isFiring() {
        return firing;
    }

    public boolean isShooterOn() {
        return shooterOn;
    }

    public NEW_SHOOTER_TRANSFER(HardwareMap hw) {
        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2 = hw.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hw.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hw.get(Servo.class, "gate");
        setShooterPIDF(SHOOTER_P, SHOOTER_I, SHOOTER_D, SHOOTER_F);
    }

    public void increaseTargetVelocity() {
        targetVelocity += VELOCITY_STEP;
        if (shooterOn) setShooterVelocity(targetVelocity);
    }

    public void decreaseTargetVelocity() {
        targetVelocity = Math.max(0, targetVelocity - VELOCITY_STEP);
        if (shooterOn) setShooterVelocity(targetVelocity);
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    // --- PIDF tuning hooks ---

    public void setShooterPIDF(double p, double i, double d, double f) {
        // Both shooters run their own velocity loop on the same target, so they get the same PIDF
        for (DcMotorEx motor : new DcMotorEx[]{shooter1, shooter2}) {
            motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(p, i, d, f));
        }
    }

    public PIDFCoefficients getShooterPIDF() {
        return shooter1.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public PIDFCoefficients getShooter2PIDF() {
        return shooter2.getPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void setShooterVelocity(double ticksPerSecond) {
        shooter1.setVelocity(ticksPerSecond);
        shooter2.setVelocity(ticksPerSecond);
    }

    // Speeds are read as absolute values, because the encoder reads negative for positive power on this robot
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

    private void spinUp() {
        shooterOn = true;
        setShooterVelocity(targetVelocity);
    }

    private void spinDown() {
        shooterOn = false;
        setShooterVelocity(0);
    }

    public Command rev() {
        return instant(() -> spinUp()).requiring(this);
    }

    public Command stopRev() {
        return instant(() -> spinDown()).requiring(this);
    }

    public Command stop() {
        return instant(() -> spinDown()).requiring(this);
    }

    public boolean isShooterReady() {

        if (!shooterOn || targetVelocity <= 0) {
            readySince = -1;
            return false;
        }

        double velocity1 = getShooterVelocity();
        double velocity2 = getShooter2Velocity();

        boolean shooter1Ready =
                Math.abs(targetVelocity - velocity1) <= READY_TOLERANCE;

        boolean shooter2Ready =
                Math.abs(targetVelocity - velocity2) <= READY_TOLERANCE;

        if (shooter1Ready && shooter2Ready) {

            if (readySince == -1) {
                readySince = System.currentTimeMillis();
            }

            return System.currentTimeMillis() - readySince
                    >= READY_STABLE_TIME;

        } else {
            readySince = -1;
            return false;
        }
    }

    public Command fire(double transferTime) {

        if (!shooterOn) {
            return instant(() -> firing = false).requiring(this);
        }

        return sequential(

                instant(() -> {
                    firing = true;
                    transfer.setPower(0);
                }),

                // Wait for both flywheels to stabilize
                waitUntil(() -> isShooterReady()),

                // Open gate
                openGate(),

                waitMs(500),

                // Feed all four pollen
                instant(() -> transfer.setPower(1.0)),

                waitMs(transferTime),

                // Stop feeding
                instant(() -> transfer.setPower(0)),

                // Close gate
                closeGate(),

                stopRev(),

                // Finish
                instant(() -> firing = false)

        ).requiring(this);
    }

    public Command transfer(double transferSpeed) {
        return instant(() -> transfer.setPower(transferSpeed)).requiring(this);
    }

    public Command transferStop() {
        return instant(() -> {
            transfer.setPower(0);
        }).requiring(this);
    }

    public void newServoPose(double newServoPose) {
        this.servoPose = newServoPose;
        this.gate.setPosition(this.servoPose);
    }
    public double getServoPose() { return this.servoPose; }
    public Command servoTuning() {
        return instant(() -> {
            gate.setPosition(servoPose);
        }).requiring(this);
    }

    public Command openGate() {
        return instant(() -> {
            gate.setPosition(0.3);
        }).requiring(this);
    }

    public Command closeGate() {
        return instant(() -> {
            gate.setPosition(0.5);
        }).requiring(this);
    }

}
