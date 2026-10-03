package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;

public abstract class AutoAimTeleOp extends OpMode {

    private Follower follower;

    private IntakeCode intake = new IntakeCode();
    private Shooter_Transfer shooter_transfer = new Shooter_Transfer();

    private int shooterSpeed = 3;

    // Auto-aim tuning
    private static final double AIM_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.7;

    // Hive coordinates
    // Red:  X = 58
    // Blue: X = 83
    private final double hiveX;

    private final double LOWER_HIVE_Y = 55;
    private final double UPPER_HIVE_Y = 84;

    public AutoAimTeleOp(double hiveX) {
        this.hiveX = hiveX;
    }

    @Override
    public void init() {

        follower = Constants.create(hardwareMap);

        intake.init(hardwareMap);
        shooter_transfer.init(hardwareMap);
    }

    @Override
    public void start() {

        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {

        Pose robotPose = follower.pose();

        // auto aim

        double turnPower;

        if (gamepad1.right_bumper) {

            // Choose whichever hive is closest to the robot's Y position
            double targetY;

            if (Math.abs(robotPose.y() - LOWER_HIVE_Y)
                    < Math.abs(robotPose.y() - UPPER_HIVE_Y)) {

                targetY = LOWER_HIVE_Y;

            } else {

                targetY = UPPER_HIVE_Y;
            }

            // Calculate angle
            double targetHeading = Math.atan2(
                    targetY - robotPose.y(),
                    hiveX - robotPose.x()
            );

            // Calculate shortest angular error
            double headingError = angleWrap(
                    targetHeading - robotPose.heading()
            );

            // Proportional controller
            turnPower = headingError * AIM_KP;

            // Limit maximum turning speed
            turnPower = clip(
                    turnPower,
                    -MAX_TURN_POWER,
                    MAX_TURN_POWER
            );

            telemetry.addLine("AUTO AIM: ON");
            telemetry.addData("Target Hive", "(%.1f, %.1f)",
                    hiveX, targetY);
            telemetry.addData("Target Heading",
                    "%.1f°", Math.toDegrees(targetHeading));
            telemetry.addData("Heading Error",
                    "%.1f°", Math.toDegrees(headingError));

        } else {

            // Normal driver-controlled rotation
            turnPower = -gamepad1.right_stick_x;

            telemetry.addLine("AUTO AIM: OFF");
        }

         // FIELD CENTRIC DRIVE


        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower,
                robotPose.heading()
        );

        follower.manual(powers);


         //   SHOOTER / INTAKE


        shooter_transfer.loop();

        intake.setIntakeSpeed(
                gamepad1.right_trigger - gamepad1.left_trigger
        );

        if (gamepad2.right_trigger > 0.1) {
            shooter_transfer.shooterState = shooterSpeed;
        } else {
            shooter_transfer.shooterState = 0;
        }

        if (gamepad2.a) {
            shooter_transfer.servoState = 1;
        } else {
            shooter_transfer.servoState = 0;
        }

        if (gamepad2.dpadUpWasPressed()) {
            if (shooterSpeed < 5) {
                shooterSpeed++;
            }
        }

        if (gamepad2.dpadDownWasPressed()) {
            if (shooterSpeed > 0) {
                shooterSpeed--;
            }
        }


         // RELOCALIZE


        if (gamepad1.start) {

            Pose cornerPose = new Pose(
                    10.5,
                    10.5,
                    Math.toRadians(90)
            );

            follower.setPose(cornerPose);
        }


         // UPDATE

        follower.update();

        robotPose = follower.pose();

        telemetry.addData("Robot X", "%.1f", robotPose.x());
        telemetry.addData("Robot Y", "%.1f", robotPose.y());
        telemetry.addData(
                "Robot Heading",
                "%.1f°",
                Math.toDegrees(robotPose.heading())
        );

        telemetry.addData("Shooter Speed", shooterSpeed);
    }

    /*
     * Wrap angle to -PI ... +PI
     */
    private double angleWrap(double angle) {

        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }

        return angle;
    }

    /*
     * Limit a value between min and max
     */
    private double clip(double value, double min, double max) {

        return Math.max(min, Math.min(max, value));
    }
}