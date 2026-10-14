package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.RedHiveTipDetector;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;
import org.firstinspires.ftc.teamcode.subSystems.FlowerIntake;

import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;


@Autonomous
public class Experimental_RED_Auto extends OpMode {
    private IntakeCode intake;
    private FlowerIntake arm;
    private Shooter_Transfer shooter;
    private Follower follower;
    private RedHiveTipDetector hiveDetector;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(32.41843971631206, 8.4, 90);
    private final Pose firstShots = p.of(36.28747044917258, 18.569976359338057, 55);
    private final Pose goToGarden = p.of(22.787234042553198, 8.210401891252953, 180);
    private final Pose intakeFromGarden = p.of(8.022458628841607, 7.868794326241126, 180);
    private final Pose doubleSureTip = p.of(36.60579196217493, 18.397163120567363, 55);
    private final Pose doubleSureTipControl = p.of(22.6193853427896, 23.5307328605201, 0);
    private final Pose goShootOtherSide = p.of(46.5981, 120.409, 90);
    private final Pose goShootOtherSideControl1 = p.of(35.7518, 57.0248, 0);
    private final Pose goShootOtherSideControl2 = p.of(21.143, 117.4232, 0);
    private final Pose goShootOtherSideSegment1Start = p.of(46.5981, 120.409, 55);
    private final Pose goShootOtherSideSegment1End = p.of(46.5981, 120.409, 330);
    private final Pose goShootOtherSideSegment2Start = p.of(46.5981, 120.409, -30);
    private final Pose goShootOtherSideSegment2End = p.of(46.5981, 120.409, 320);
    private final Pose goShootOtherSideSegment3Start = p.of(46.5981, 120.409, -40);
    private final Pose goShootOtherSideSegment3End = p.of(46.5981, 120.409, 310);
    private final Pose goShootOtherSideSegment4Start = p.of(46.5981, 120.409, -50);
    private final Pose goShootOtherSideSegment4End = p.of(46.5981, 120.409, 300);
    private final Pose goShootOtherSideSegment5Start = p.of(46.5981, 120.409, -60);
    private final Pose goShootOtherSideSegment5End = p.of(46.5981, 120.409, 90);
    private final Pose intakeFromFlower = p.of(46.92198581560283, 130.15602836879435, 90);
    private final Pose farShoot = p.of(46.8227, 114.2234, -70);

    //Idk what these farShootSegments that pedro gave me are but I'll keep them just in case they are important
    private final Pose farShootSegment1Start = p.of(46.8227, 114.2234, 90);
    private final Pose farShootSegment1End = p.of(46.8227, 114.2234, 90);
    private final Pose farShootSegment2Start = p.of(46.8227, 114.2234, 90);
    private final Pose farShootSegment2End = p.of(46.8227, 114.2234, 290);
    private final Pose goParkStart = p.of(46.8227, 114.2234, 300);
    private final Pose goPark = p.of(14.591, 95.708, 180);
    private final Pose goParkControl = p.of(39.961, 95.4551, 0);




    private Path goShoot1() {
        return line(startPose,firstShots).linear(startPose,firstShots);
    }
    private Path goIntake(){return line(firstShots,goToGarden).linear(firstShots,goToGarden);}
    private Path closeIntake(){
        return line(goToGarden,intakeFromGarden).linear(goToGarden,intakeFromGarden);
    }
    private Path goShootCloseAgain(){return curve(intakeFromGarden,doubleSureTipControl, doubleSureTip).linear(intakeFromGarden,doubleSureTip);}
    private Path goShootFar() {
        return curve(doubleSureTip,goShootOtherSideControl1,goShootOtherSideControl2,goShootOtherSide).heading(Interpolator.piecewise().until(0.7189, Interpolator.linear(goShootOtherSideSegment1Start, goShootOtherSideSegment1End)).until(0.8955, Interpolator.linear(goShootOtherSideSegment2Start, goShootOtherSideSegment2End)).until(0.9434, Interpolator.linear(goShootOtherSideSegment3Start, goShootOtherSideSegment3End)).until(0.9999, Interpolator.linear(goShootOtherSideSegment4Start, goShootOtherSideSegment4End)).until(1, Interpolator.linear(goShootOtherSideSegment5Start, goShootOtherSideSegment5End)));
    }
    private Path intakeFromFlower(){
        return line(goShootOtherSide,intakeFromFlower).linear(goShootOtherSide,intakeFromFlower);
    }
    private Path farShoot(){
        return line(intakeFromFlower,farShoot).linear(intakeFromFlower,farShoot);
    }
    private Path goPark(){
        return curve(farShoot,goParkControl,goPark).linear(farShoot,goPark);
    }

    private Command shootOne(boolean waitForNextShot) {
        double time_between_shots = 250;
        double flywheel_spinup_time = 50;

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

    private Command shootFour() {
        return sequential(
                shootOne(true),
                shootOne(true),
                shootOne(true),
                shootOne(false)
        );
    }

    private Command shootHive() {

        return sequential(

                // only shoot if the hive tags are visible
                conditional(
                        () -> hiveDetector.areHiveTagsVisible(),

                        // tags visible, take the first shot
                        sequential(
                                shootOne(false),

                                // give the hive time to tip
                                waitMs(3000),

                                // check again before the second shot
                                conditional(
                                        () -> hiveDetector.areHiveTagsVisible(),

                                        sequential(
                                                shootOne(false),
                                                waitMs(3000),

                                                // check before the final shot
                                                conditional(
                                                        () -> hiveDetector.areHiveTagsVisible(),

                                                        // still visible: final attempt
                                                        shootOne(false),

                                                        // tags disappeared: don't shoot
                                                        instant(() -> {})
                                                )
                                        ),

                                        // tags disappeared = don't shoot
                                        instant(() -> {})
                                )
                        ),

                        // tags not visible initially = don't shoot at all
                        instant(() -> {})
                )
        );
    }

    private Command goShootFarAndShootFour() {
        return parallel(
                follow(follower, goShootFar()),

                sequential(
                        // Start shooting when within 16 inches
                        // of the path's endpoint.
                        waitUntil(() ->
                                Math.hypot(
                                        follower.pose().x() - goShootOtherSide.x(),
                                        follower.pose().y() - goShootOtherSide.y()
                                ) < 16
                        ),

                        shootFour()
                )
        );
    }

    private Command autoRoutine() {
        double time_between_shots = 250;
        double flywheel_spinup_time = 800;
        return sequential(
                instant(() -> arm.raiseArm()),

                instant(() -> Shooter_Transfer.shooterState = 2),

                waitMs(3000),

                shootOne(true),
                shootOne(true),
                shootOne(true),
                shootOne(false),

                instant(() -> intake.run(1.0)),

                follow(follower, goIntake()),

                follow(follower, closeIntake()),

                instant(() -> intake.stop()),

                follow(follower, goShootCloseAgain()),

                shootHive(),

                goShootFarAndShootFour(),

                instant(() -> arm.lowerArm()),

                instant(() -> intake.run(1.0)),

                follow(follower, intakeFromFlower()),

                waitMs(500),

                instant(() -> intake.stop()),

                follow(follower, farShoot()),

                instant(() -> arm.raiseArm()),

                shootHive(),

                instant(() -> intake.stop()),

                follow(follower, goPark())
        );
    }


    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
        intake = new IntakeCode(hardwareMap);
        arm = new FlowerIntake(hardwareMap);
        shooter = new Shooter_Transfer();
        shooter.init(hardwareMap);

        hiveDetector = new RedHiveTipDetector();
        hiveDetector.init(hardwareMap);

        Shooter_Transfer.servoState = 0;
        shooter.loop();

        hiveDetector = new RedHiveTipDetector();
        hiveDetector.init(hardwareMap);

        instant(() -> arm.raiseArm());
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
