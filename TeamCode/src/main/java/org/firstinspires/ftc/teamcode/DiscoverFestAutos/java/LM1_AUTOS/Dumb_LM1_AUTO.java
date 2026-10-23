package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java.LM1_AUTOS;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.*;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "LM1 Main")
public class Dumb_LM1_AUTO extends OpMode {

    private IntakeCode intake;
    private NEW_SHOOTER_TRANSFER newShooter;
    private Follower follower;
    private FlowerIntake arm;
    private RedHiveTipDetector hiveDetector;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55.5, 8.4, 270);
    private final Pose goParkStart = poseFactory.of(55.5, 8.4, 270);
    private final Pose goPark = poseFactory.of(13.5479, 95.2776, 180);
    private final Pose goParkControl1 = poseFactory.of(13.1288, 39.474, 0);
    private final Pose goParkControl2 = poseFactory.of(37.4882, 64.4303, 0);

    public Path goPark() {
        return curve(goParkStart, goParkControl1, goParkControl2, goPark).linear(goParkStart, goPark);
    }


    private Command shootHive() {

        return sequential(

                // only shoot if the hive tags are visible
                conditional(
                        () -> hiveDetector.areHiveTagsVisible(),
                        newShooter.fire(3000),
                        instant(() -> {})
                )
        );
    }



    private Command autoRoutine() {
        return sequential(
                newShooter.rev(),
                waitMs(1000),
                newShooter.fire(2500),
                follow(follower, goPark())
        );

    }



    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        intake = new IntakeCode(hardwareMap);

        newShooter = new NEW_SHOOTER_TRANSFER(hardwareMap);
        arm = new FlowerIntake(hardwareMap);

        hiveDetector = new RedHiveTipDetector();
        hiveDetector.init(hardwareMap);

        hiveDetector = new RedHiveTipDetector();
        hiveDetector.init(hardwareMap);
        newShooter.closeGate().schedule();
        arm.raiseArm().schedule();

        newShooter.stop();
        Scheduler.execute();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }

    @Override
    public void stop() {
        OpModeStorage.autonomousEndPose = follower.pose(); //saves your position in that file
    }

}