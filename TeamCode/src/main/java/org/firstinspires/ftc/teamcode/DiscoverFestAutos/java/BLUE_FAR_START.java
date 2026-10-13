package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;
import org.firstinspires.ftc.teamcode.subSystems.BlueHiveTipDetector;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.commands.Commands.*;


@Autonomous
public class BLUE_FAR_START extends OpMode {
    private IntakeCode intake;
    private Shooter_Transfer shooter;
    private Follower follower;
    private BlueHiveTipDetector hiveDetector;
    private boolean waitForHive = false;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(59.67966903073285, 133.77777777777783, 270);
    private final Pose goIntakeUnderBasket = p.of(58.34160756501182, 84.839243498818, 270);
    private final Pose goBackToShoot = p.of(59.56973995271868, 134.29432624113474, 270);
    private final Pose goPark = p.of(12.02955082742317, 116.23758865248227, 0);

    private final Pose goParkControl = p.of(53.37706855791962, 93.70449172576832, 270);

    private Path goIntakeUnderBasket(){
        return line(startPose,goIntakeUnderBasket).linear(startPose,goIntakeUnderBasket);
    }
    private Path goBackToShoot(){
        return line(goIntakeUnderBasket,goBackToShoot).linear(goIntakeUnderBasket,goBackToShoot);
    }
    private Path goPark(){
        return curve(goBackToShoot,goParkControl,goPark).linear(goBackToShoot,goPark);
    }

    private Command waitForHiveTags() {
        return waitUntil(
                () -> hiveDetector.areHiveTagsVisible()
        );
    }

    private Command shootOne(boolean waitForNextShot) {
        double time_between_shots = 250;
        double flywheel_spinup_time = 800;

        return sequential(
                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                conditional(
                        () -> waitForNextShot,
                        waitMs(flywheel_spinup_time),
                        instant(() -> {})
                )
        );
    }




    private Command autoRoutine() {
        return sequential(

                waitForHiveTags(),

                instant(() -> Shooter_Transfer.shooterState = 2),

                waitMs(4000),

                shootOne(true),
                shootOne(true),
                shootOne(true),
                shootOne(false),

                instant(() -> Shooter_Transfer.shooterState = 0),

                instant(() -> intake.run(1.0)),

                follow(follower, goIntakeUnderBasket()),

                follow(follower, goBackToShoot()),

                instant(() -> intake.stop()),

                instant(() -> Shooter_Transfer.shooterState = 2),

                waitMs(2000),

                shootOne(true),
                shootOne(true),
                shootOne(true),
                shootOne(false),

                instant(() -> Shooter_Transfer.shooterState = 0),

                follow(follower, goPark())
        );
    }


    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new IntakeCode(hardwareMap);

        shooter = new Shooter_Transfer();
        shooter.init(hardwareMap);

        Shooter_Transfer.servoState = 0;
        Shooter_Transfer.shooterState = 0;
        shooter.loop();
    }

    @Override
    public void start() {
        schedule(autoRoutine());
    }


    @Override
    public void loop() {
        shooter.loop();
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
