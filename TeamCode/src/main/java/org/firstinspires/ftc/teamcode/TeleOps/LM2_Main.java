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
import org.firstinspires.ftc.teamcode.subSystems.NEW_SHOOTER_TRANSFER;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;

@TeleOp(name = "LM2 TeleOp")
public class LM2_Main extends OpMode {

    private static final double AIM_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.7;
    private static final double HIVE_X = 58.0;
    private static final double LOWER_HIVE_Y = 55.0;
    private static final double UPPER_HIVE_Y = 84.0;
    private static final double SHOT_FLIGHT_TIME = 0.7;
    private static final double INTAKE_DEADBAND = 0.0;
    private static final double SHOOTER_TRIGGER_DEADBAND = 0.1;
    private static final double MANUAL_TRANSFER_POWER = 0.5;
    private static final double FIRE_TRANSFER_TIME_MS = 3000;
    private static final Pose LOCALIZATION_RESET_POSE =
            new Pose(10.5, 10.5, Math.toRadians(90));

    private Follower follower;
    private IntakeCode intake;
    private FlowerIntake arm;
    private NEW_SHOOTER_TRANSFER shooter;
    private boolean armRaised = true;

    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        intake = new IntakeCode(hardwareMap);
        arm = new FlowerIntake(hardwareMap);
        shooter = new NEW_SHOOTER_TRANSFER(hardwareMap);

        shooter.stop().schedule();
        Scheduler.execute();
    }

    @Override
    public void start() {
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
        arm.raiseArm().schedule();
        shooter.newServoPose(NEW_SHOOTER_TRANSFER.GATE_CLOSED_POSITION);
        Scheduler.execute();
    }

    @Override
    public void loop() {
        updateDrive();
        updateIntake();
        updateArm();
        updateShooter();
        Scheduler.execute();
    }

    private void updateDrive() {
        if (gamepad1.startWasPressed()) {
            follower.setPose(LOCALIZATION_RESET_POSE);
        }
        follower.update();

        Pose pose = follower.pose();
        double turnPower = gamepad1.right_bumper
                ? calculateAimPower(pose)
                : -gamepad1.right_stick_x;
        if (!gamepad1.right_bumper) {
            telemetry.addLine("AUTO AIM: OFF");
        }

        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower,
                pose.heading()
        );
        follower.manual(powers);

        pose = follower.pose();
        telemetry.addData("Robot X", pose.x());
        telemetry.addData("Robot Y", pose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(pose.heading()));
    }

    private double calculateAimPower(Pose pose) {
        double targetY = Math.abs(pose.y() - LOWER_HIVE_Y)
                < Math.abs(pose.y() - UPPER_HIVE_Y)
                ? LOWER_HIVE_Y
                : UPPER_HIVE_Y;

        double vx = follower.velocity().vx;
        double vy = follower.velocity().vy;
        double leadX = HIVE_X - pose.x() - vx * SHOT_FLIGHT_TIME;
        double leadY = targetY - pose.y() - vy * SHOT_FLIGHT_TIME;
        double targetHeading = Math.atan2(leadY, leadX);
        double headingError = angleWrap(targetHeading - pose.heading());

        telemetry.addLine("AUTO AIM: ON");
        telemetry.addData("Target Hive", "(%.1f, %.1f)", HIVE_X, targetY);
        telemetry.addData("Heading Error", "%.1f degrees", Math.toDegrees(headingError));

        return clip(headingError * AIM_KP, -MAX_TURN_POWER, MAX_TURN_POWER);
    }

    private void updateIntake() {
        double intakePower = gamepad1.right_trigger - gamepad1.left_trigger;
        intake.run(intakePower).schedule();
        telemetry.addLine(intakePower > INTAKE_DEADBAND ? "Intake Active" : "Intake Not Active");
    }

    private void updateArm() {
        if (gamepad1.triangle) {
            setArmRaised(true);
            telemetry.addLine("Raising Arm");
        }

        if (gamepad1.crossWasPressed()) {
            setArmRaised(false);
            telemetry.addLine("Lowering Arm");
        }

        if (gamepad1.leftBumperWasPressed()) {
            setArmRaised(!armRaised);
            telemetry.addLine(armRaised ? "Raising Arm (toggle)" : "Lowering Arm (toggle)");
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

    private void updateShooter() {
        boolean firing = shooter.isFiring();
        boolean revving = gamepad2.right_trigger > SHOOTER_TRIGGER_DEADBAND;

        if (!firing) {
            (gamepad2.square ? shooter.openGate() : shooter.closeGate()).schedule();

            if (revving != shooter.isShooterOn()) {
                (revving ? shooter.rev() : shooter.stop()).schedule();
            }

            if (gamepad2.left_trigger > SHOOTER_TRIGGER_DEADBAND) {
                shooter.transfer(MANUAL_TRANSFER_POWER).schedule();
                telemetry.addLine("Transfer Active");
            } else {
                shooter.transferStop().schedule();
                telemetry.addLine("Transfer Inactive");
            }

            if (gamepad2.crossWasPressed()) {
                shooter.fire(FIRE_TRANSFER_TIME_MS).schedule();
            }
        }

        if (gamepad2.dpadUpWasPressed()) {
            shooter.increaseTargetVelocity();
        }
        if (gamepad2.dpadDownWasPressed()) {
            shooter.decreaseTargetVelocity();
        }

        boolean ready = shooter.isShooterReady();
        if (ready) {
            gamepad2.rumble(1.0, 1.0, 100);
        } else {
            gamepad2.stopRumble();
        }

        telemetry.addLine(revving ? "Flywheel Active" : "Flywheel Inactive");
        String shooterStatus = ready
                ? "READY!"
                : shooter.isShooterOn() ? "SPINNING UP" : "OFF";
        telemetry.addData("Shooter Ready", shooterStatus);
        telemetry.addData("Target Velocity", shooter.getTargetVelocity());
        telemetry.addData("Servo Position", shooter.getServoPose());
        telemetry.addData("Shooter1 Speed", shooter.getShooterVelocity());
        telemetry.addData("Shooter2 Speed", shooter.getShooter2Velocity());
    }

    private double angleWrap(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    private double clip(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
