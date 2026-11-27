
package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;

import org.firstinspires.ftc.teamcode.subSystems.NewSubSystems.Shooter;
import org.firstinspires.ftc.teamcode.subSystems.NewSubSystems.ShooterConstants;
import org.firstinspires.ftc.teamcode.subSystems.NewSubSystems.ShotSolution;

@TeleOp(name = "League Meet 4 TeleOp")
public class LM4_Main extends OpMode {

    private static final double AIM_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.7;

    private static final double INTAKE_DEADBAND = 0.0;
    private static final double TRIGGER_DEADBAND = 0.1;

    private static final double MANUAL_TRANSFER_POWER = 0.5;
    private static final double FIRE_TRANSFER_TIME_MS = 1000;

    private static final double VELOCITY_STEP = 10;

    private static final Pose RESET_POSE =
            new Pose(10.5, 10.5, Math.toRadians(90));

    private static final long SERVO_SETTLE_TIME_MS = 300;

    private Follower follower;

    private IntakeCode intake;
    private FlowerIntake arm;

    private ShooterConstants shooterHardware;
    private Shooter shooter;

    private boolean armRaised = true;

    private double manualVelocityOffset = 0;

    private long lastAutoAimTime = 0;

    private static final long AUTO_AIM_HOLD_MS = 5000;

    private double lastAimError = 0;

    @Override
    public void init() {

        Scheduler.reset();

        follower = Constants.create(hardwareMap);

        intake = new IntakeCode(hardwareMap);

        arm = new FlowerIntake(hardwareMap);

        shooterHardware = new ShooterConstants(hardwareMap);

        shooter = new Shooter(
                follower,
                shooterHardware
        );

        shooterHardware.stop().schedule();

        Scheduler.execute();
    }

    @Override
    public void start() {

        follower.setPose(OpModeStorage.autonomousEndPose);

        follower.update();

        arm.raiseArm().schedule();

        shooterHardware.newServoPose(
                ShooterConstants.GATE_CLOSED_POSITION
        );

        Scheduler.execute();
    }

    @Override
    public void loop() {

        follower.update();

        updateDrive();

        updateIntake();

        updateArm();

        updateShooter();

        Scheduler.execute();
    }

    // ======================================
    // DRIVE
    // ======================================

    private void updateDrive() {

        if (gamepad1.startWasPressed()) {
            follower.setPose(RESET_POSE);
        }

        Pose pose = follower.pose();

        if (gamepad1.right_bumper) {
            lastAutoAimTime = System.currentTimeMillis();
        }

        double turnPower = gamepad1.right_bumper
                ? calculateAimPower(pose)
                : -gamepad1.right_stick_x;

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower,
                pose.heading()
        );

        follower.manual(powers);

        telemetry.addData(
                "Robot X",
                "%.1f",
                pose.x()
        );

        telemetry.addData(
                "Robot Y",
                "%.1f",
                pose.y()
        );

        telemetry.addData(
                "Robot Heading",
                "%.1f",
                Math.toDegrees(pose.heading())
        );

        telemetry.addData(
                "Auto Aim",
                gamepad1.right_bumper ? "ON" : "OFF"
        );
    }

    private double calculateAimPower(Pose pose) {

        double targetY =
                Math.abs(pose.y() - 55.0)
                        < Math.abs(pose.y() - 84.0)
                        ? 55.0 : 84.0;

        double vx = follower.velocity().vx;
        double vy = follower.velocity().vy;

        double leadX = 58.0 - pose.x() - vx * 0.7;

        double leadY = targetY - pose.y() - vy * 0.7;

        double targetHeading =
                Math.atan2(leadY, leadX) + Math.PI;

        double headingError =
                angleWrap(targetHeading - pose.heading());

        lastAimError = headingError;

        return clip(
                headingError * AIM_KP,
                -MAX_TURN_POWER,
                MAX_TURN_POWER
        );
    }

    // ======================================
    // INTAKE
    // ======================================

    private void updateIntake() {

        double power =
                gamepad1.right_trigger - gamepad1.left_trigger;

        intake.run(power).schedule();

        telemetry.addData(
                "Intake",
                power > INTAKE_DEADBAND ? "Active" : "Inactive"
        );
    }

    // ======================================
    // ARM
    // ======================================

    private void updateArm() {

        if (gamepad1.triangle) {
            setArmRaised(true);
        }

        if (gamepad1.crossWasPressed()) {
            setArmRaised(false);
        }

        if (gamepad1.leftBumperWasPressed()) {
            setArmRaised(!armRaised);
        }
    }

    private void setArmRaised(boolean raised) {

        armRaised = raised;

        if (raised) {
            arm.raiseArm().schedule();
        } else {
            arm.lowerArm().schedule();
        }
    }

    // ======================================
    // SHOOTER
    // ======================================

    private void updateShooter() {

        boolean firing = shooterHardware.isFiring();

        boolean revving =
                gamepad2.right_trigger > TRIGGER_DEADBAND;

        boolean firePressed = gamepad2.crossWasPressed();

        // Calculate all three shooting targets.
        ShotSolution solution = shooter.calculateShot();

        // Apply calculated targets.
        shooter.updateShooter(solution);

        // Manual flywheel adjustment.
        if (gamepad2.dpadUpWasPressed()) {
            manualVelocityOffset += VELOCITY_STEP;
        }

        if (gamepad2.dpadDownWasPressed()) {
            manualVelocityOffset -= VELOCITY_STEP;
        }

        shooterHardware.setTargetVelocity(
                solution.flywheelVelocity + manualVelocityOffset
        );

        // Don't interfere with an active firing sequence.
        if (!firing) {

            (gamepad2.square
                    ? shooterHardware.openGate()
                    : shooterHardware.closeGate()
            ).schedule();

            if (revving != shooterHardware.isShooterOn()) {

                (revving
                        ? shooterHardware.rev()
                        : shooterHardware.stop()
                ).schedule();
            }

            if (gamepad2.left_trigger > TRIGGER_DEADBAND) {

                shooterHardware.transfer(
                        MANUAL_TRANSFER_POWER
                ).schedule();

            } else {

                shooterHardware.transferStop().schedule();
            }

            if (firePressed) {

                shooterHardware.fire(
                        FIRE_TRANSFER_TIME_MS
                ).schedule();
            }
        }

        // ==================================
        // READINESS
        // ==================================

        boolean flywheelReady =
                shooterHardware.isShooterReady();

        boolean servosSettled =
                shooterHardware.areServosSettled();

        boolean ready = flywheelReady && servosSettled;

        if (ready) {
            gamepad2.rumble(1.0, 1.0, 100);
        } else {
            gamepad2.stopRumble();
        }

        // ==================================
        // TELEMETRY
        // ==================================

        telemetry.addData(
                "Distance",
                "%.1f inches",
                solution.distance
        );

        telemetry.addData(
                "Flywheel Target",
                "%.0f",
                solution.flywheelVelocity + manualVelocityOffset
        );

        telemetry.addData(
                "Flywheel 1",
                "%.0f",
                shooterHardware.getShooterVelocity()
        );

        telemetry.addData(
                "Flywheel 2",
                "%.0f",
                shooterHardware.getShooter2Velocity()
        );

        telemetry.addData(
                "Hood Angle",
                "%.1f degrees",
                Math.toDegrees(solution.hoodAngle)
        );

        telemetry.addData(
                "Hood Position",
                "%.3f",
                shooterHardware.getHoodPosition()
        );

        telemetry.addData(
                "Turret Angle",
                "%.1f degrees",
                Math.toDegrees(solution.turretAngle)
        );

        telemetry.addData(
                "Turret Position",
                "%.3f",
                shooterHardware.getTurretPosition()
        );

        telemetry.addData(
                "Flywheel Ready",
                flywheelReady
        );

        telemetry.addData(
                "Servos Settled",
                servosSettled
        );

        telemetry.addData(
                "SHOOTER STATUS",
                ready ? "READY!" :
                        revving ? "SPINNING UP" : "OFF"
        );

        telemetry.addData(
                "Manual Offset",
                manualVelocityOffset
        );

        telemetry.addData(
                "Aim Error",
                "%.1f degrees",
                Math.toDegrees(lastAimError)
        );
    }

    // ======================================
    // UTILITIES
    // ======================================

    private double angleWrap(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    private double clip(
            double value,
            double min,
            double max
    ) {

        return Math.max(min, Math.min(max, value));
    }
}
