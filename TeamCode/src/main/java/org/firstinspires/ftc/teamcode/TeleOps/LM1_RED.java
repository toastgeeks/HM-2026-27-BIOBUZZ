package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;
import org.firstinspires.ftc.teamcode.subSystems.NEW_SHOOTER_TRANSFER;

import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

@TeleOp(name = "Red Teleop")
public class LM1_RED extends OpMode {

    private Follower follower;

    private IntakeCode intake;

    private FlowerIntake arm;

    private NEW_SHOOTER_TRANSFER newShooter;

    int shooterSpeed = 3;

    int armState = 2;

    private static final double AIM_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.7;

    private static final double HIVE_X = 58.0;
    private static final double LOWER_HIVE_Y = 55.0;
    private static final double UPPER_HIVE_Y = 84.0;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = new IntakeCode(hardwareMap);

        arm = new FlowerIntake(hardwareMap);

        newShooter = new NEW_SHOOTER_TRANSFER(hardwareMap);

        newShooter.stop();
    }


    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {

        // this NEEDS to be before Pose creation cus it needs the most accurate pose possible
        // relocalise button
        if (gamepad1.start) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose
        }    follower.update();


        Pose robotPose = follower.pose();

        double turnPower;

        if (gamepad1.right_bumper) {
            // Choose the hive closest to the robot's Y position
            double targetY;

            if (Math.abs(robotPose.y() - LOWER_HIVE_Y)
                    < Math.abs(robotPose.y() - UPPER_HIVE_Y)) {
                targetY = LOWER_HIVE_Y;
            } else {
                targetY = UPPER_HIVE_Y;
            }

            // Calculate the heading toward the red hive
            double targetHeading = Math.atan2(
                    targetY - robotPose.y(),
                    HIVE_X - robotPose.x()
            );

            // Find the shortest rotation to the target
            double headingError = angleWrap(
                    targetHeading - robotPose.heading()
            );

            // Proportional turning controller
            turnPower = clip(
                    headingError * AIM_KP,
                    -MAX_TURN_POWER,
                    MAX_TURN_POWER
            );

            telemetry.addLine("AUTO AIM: ON");
            telemetry.addData("Target Hive", "(%.1f, %.1f)",
                    HIVE_X, targetY);
            telemetry.addData("Heading Error", "%.1f degrees",
                    Math.toDegrees(headingError));

        } else {
            // Normal driver-controlled rotation
            turnPower = -gamepad1.right_stick_x;

            telemetry.addLine("AUTO AIM: OFF");
        }

        // Keep field-centric driving
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x,
                turnPower,
                robotPose.heading()
        );

        follower.manual(powers);

        robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        intake.run(gamepad1.right_trigger - gamepad1.left_trigger);

        if (armState == 2){
            instant(() -> arm.lowerArm());
        }
        else if(armState == 1) {
            instant(() -> arm.raiseArm());
        }

        if (gamepad1.aWasPressed()){
            if(armState == 1){
                armState = 2;
            }
            else {
                armState = 1;
            }
        }

        if (gamepad2.right_trigger > 0.1){
            newShooter.rev();
        }
        else {
            newShooter.stop();
        }

        if (gamepad2.a){
            newShooter.fire(500);
        }

        if (gamepad2.b){
            newShooter.fire(100);
        }

        if (gamepad2.dpadUpWasPressed()) {
            if (shooterSpeed < 5) shooterSpeed += 1;
        }
        if (gamepad2.dpadDownWasPressed()) {
            if (shooterSpeed > 0) shooterSpeed -= 1;
        }

        if (shooterSpeed == 0) {
            telemetry.addLine("Shooter Speed = 0%");
        } else if (shooterSpeed == 1) {
            telemetry.addLine("Shooter Speed = 55%");
        } else if (shooterSpeed == 2) {
            telemetry.addLine("Shooter Speed = 60%");
        } else if (shooterSpeed == 3) {
            telemetry.addLine("Shooter Speed = 65%");
        } else if (shooterSpeed == 4) {
            telemetry.addLine("Shooter Speed = 70%");
        } else if (shooterSpeed == 5) {
            telemetry.addLine("Shooter Speed = 75%");
        }

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