package org.firstinspires.ftc.teamcode.TeleOps;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;

@TeleOp(name = "Discoverfest TeleOp")
public class DiscoverfestTeleOp extends OpMode {

    private Follower follower;

    IntakeCode intake = new IntakeCode();

    Shooter_Transfer shooter_transfer = new Shooter_Transfer();

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();
    }

    @Override
    public void loop() {
        DrivePowers powers = ManualDrive.fieldCentric(
                -gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                gamepad1.right_stick_x,
                follower.pose().heading()
        );
        follower.manual(powers);

        // relocalise button
        if (gamepad1.start) {
            Pose cornerPose = new Pose(10.5, 10.5, Math.toRadians(90));
            // On the fly Pose creation, we dont recommend this for Autonomous. Only accepts radians for heading
            follower.setPose(cornerPose); // overrides our pose  
        }

        follower.update();
        Pose robotPose = follower.pose(); // returns a Pose object
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));

        intake.setIntakeSpeed(gamepad1.right_trigger - gamepad1.left_trigger);

        if (gamepad2.right_trigger > 0.1){
            shooter_transfer.shooterState = 1;
        }
        else {
            shooter_transfer.shooterState = 0;
        }

        if (gamepad2.a){
            shooter_transfer.servoState = 1;
        }
        else {
            shooter_transfer.servoState = 0;
        }
    }

}