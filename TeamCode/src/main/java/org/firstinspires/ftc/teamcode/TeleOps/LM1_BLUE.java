package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
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

@TeleOp(name = "BLUE Teleop")
public class LM1_BLUE extends OpMode {

    private Follower follower;

    private IntakeCode intake;

    private FlowerIntake arm;

    private NEW_SHOOTER_TRANSFER newShooter;

    private NEW_SHOOTER_TRANSFER servo;

    int armState = 2;

    private static final double AIM_KP = 1.5;
    private static final double MAX_TURN_POWER = 0.7;

    private static final double HIVE_X = 83.0;
    private static final double LOWER_HIVE_Y = 55.0;
    private static final double UPPER_HIVE_Y = 84.0;


    @Override
    public void init() {
        follower = Constants.create(hardwareMap);

        intake = new IntakeCode(hardwareMap);

        arm = new FlowerIntake(hardwareMap);

        newShooter = new NEW_SHOOTER_TRANSFER(hardwareMap);

        newShooter.stop().schedule();

        newShooter.newServoPose(0.6);

        arm.lowerArm().schedule();

        Scheduler.execute();
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

        intake.run(gamepad1.right_trigger - gamepad1.left_trigger).schedule();
//        newShooter.transfer(gamepad1.right_trigger - gamepad1.left_trigger).schedule();
        newShooter.transfer(gamepad2.left_trigger).schedule();

        if (gamepad2.square){
            newShooter.openGate().schedule();
        }
        else {
            newShooter.closeGate().schedule();
        }

        if (gamepad1.right_trigger - gamepad1.left_trigger > 0) {
            telemetry.addLine("Intake Active");
        }
        else {
            telemetry.addLine("Intake Not Active");
        }


        if (gamepad1.triangle) {
            arm.raiseArm().schedule();
        }

        if (gamepad1.cross) {
            arm.lowerArm().schedule();
        }

        if (gamepad2.right_trigger > 0.1){
            newShooter.rev().schedule();
            telemetry.addLine("Flywheel Active");
        }
        else {
            newShooter.stop().schedule();
            telemetry.addLine("Flywheel Inactive");
        }

        if (gamepad2.left_trigger > 0.1){
            newShooter.transfer(0.6).schedule();
            telemetry.addLine("Transfer Active");
        }
        else {
            newShooter.transferStop().schedule();
            telemetry.addLine("Transfer Inactive");
        }

        if (gamepad2.cross){
            //newShooter.fire(100).schedule();
            telemetry.addLine("Firing One");
        }
        else {
            telemetry.addLine("Not Firing One");
        }


        if(gamepad2.dpadUpWasPressed()) {
            newShooter.increaseTargetVelocity();
        }
        if (gamepad2.dpadDownWasPressed()) {
            newShooter.decreaseTargetVelocity();
        }


        telemetry.addData("Servo Position", newShooter.getServoPose());
        telemetry.addData("Shooter1 Speed", newShooter.getShooterVelocity());
        telemetry.addData("Shooter2 Speed", newShooter.getShooter2Velocity());

        Scheduler.execute();

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