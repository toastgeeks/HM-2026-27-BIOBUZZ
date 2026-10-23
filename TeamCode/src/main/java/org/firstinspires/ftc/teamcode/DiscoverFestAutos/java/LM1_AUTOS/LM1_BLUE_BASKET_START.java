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

@Autonomous(name = "Blue Basket Auto")
public class LM1_BLUE_BASKET_START extends OpMode {

    private IntakeCode intake;
    private NEW_SHOOTER_TRANSFER newShooter;
    private Follower follower;
    private FlowerIntake arm;
    private BlueHiveTipDetector hiveDetector;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(32.4184, 8.4, 90);
    private final Pose firstShotsStart = poseFactory.of(32.4184, 8.4, 270);
    private final Pose firstShots = poseFactory.of(36.2875, 18.57, 235);
    private final Pose goToGarden = poseFactory.of(22.7872, 8.2104, 180);
    private final Pose intakeFromGarden = poseFactory.of(8.0225, 7.8688, 180);
    private final Pose doubleSureTip = poseFactory.of(36.6058, 18.3972, 235);
    private final Pose doubleSureTipControl1 = poseFactory.of(22.6194, 23.5307, 0);
    private final Pose goShootOtherSide = poseFactory.of(46.9326, 120.409, 105);
    private final Pose goShootOtherSideControl1 = poseFactory.of(35.7518, 57.0248, 0);
    private final Pose goShootOtherSideControl2 = poseFactory.of(21.143, 117.4232, 0);
    private final Pose intakeFromFlower = poseFactory.of(47.1253, 125.2553, 90);
    private final Pose shootAfterFlower = poseFactory.of(46.8333, 120.5615, 105);
    private final Pose goPark = poseFactory.of(13.2134, 116.0176, 0);
    private final Pose goParkControl1 = poseFactory.of(31.5272, 112.7329, 0);

    public Path firstShots() {
        return line(firstShotsStart, firstShots).linear(firstShotsStart, firstShots);
    }

    public Path goToGarden() {
        return line(firstShots, goToGarden).linear(firstShots, goToGarden);
    }

    public Path intakeFromGarden() {
        return line(goToGarden, intakeFromGarden).constant(intakeFromGarden);
    }

    public Path doubleSureTip() {
        return curve(intakeFromGarden, doubleSureTipControl1, doubleSureTip).linear(intakeFromGarden, doubleSureTip);
    }

    public Path goShootOtherSide() {
        return curve(doubleSureTip, goShootOtherSideControl1, goShootOtherSideControl2, goShootOtherSide).linear(doubleSureTip, goShootOtherSide);
    }

    public Path intakeFromFlower() {
        return line(goShootOtherSide, intakeFromFlower).linear(goShootOtherSide, intakeFromFlower);
    }

    public Path shootAfterFlower() {
        return line(intakeFromFlower, shootAfterFlower).linear(intakeFromFlower, shootAfterFlower);
    }

    public Path goPark() {
        return curve(shootAfterFlower, goParkControl1, goPark).linear(shootAfterFlower, goPark);
    }


    //private Command shootHive() {
//
    //    return sequential(
//
    //            // only shoot if the hive tags are visible
    //            conditional(
    //                    () -> hiveDetector.areHiveTagsVisible(),
//
    //                    // tags visible: take the first shot
    //                    sequential(
    //                            //newShooter.fire(100),
//
    //                            // give the hive time to tip
    //                            waitMs(3000),
//
    //                            // check again before the second shot
    //                            conditional(
    //                                    () -> hiveDetector.areHiveTagsVisible(),
//
    //                                    sequential(
    //                                            newShooter.fire(100),
    //                                            waitMs(3000),
//
    //                                            // check before the final shot
    //                                            conditional(
    //                                                    () -> hiveDetector.areHiveTagsVisible(),
//
    //                                                    // still visible: final attempt
    //                                                    newShooter.fire(100),
//
    //                                                    // tags disappeared: don't shoot
    //                                                    instant(() -> {})
    //                                            )
    //                                    ),
//
    //                                    // tags disappeared: don't shoot
    //                                    instant(() -> {})
    //                            )
    //                    ),
//
    //                    // tags not visible initially: don't shoot at all
    //                    instant(() -> {})
    //            )
    //    );
    //}



    private Command autoRoutine() {
        return sequential(
                newShooter.rev(),
                waitMs(1000),
                follow(follower, firstShots()),
              //  newShooter.fire(400),
                intake.run(1),
                follow(follower, goToGarden()),
                follow(follower, intakeFromGarden()),
                intake.run(0),
                follow(follower, doubleSureTip()),
                //shootHive(),
                follow(follower, goShootOtherSide()),
              //  shootHive(),
                instant(() -> arm.lowerArm()),
                follow(follower, intakeFromFlower()),
                follow(follower, shootAfterFlower()),
                instant(() -> arm.raiseArm()),
                //shootHive(),
                newShooter.stopRev(),
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

        hiveDetector = new BlueHiveTipDetector();
        hiveDetector.init(hardwareMap);

        hiveDetector = new BlueHiveTipDetector();
        hiveDetector.init(hardwareMap);

        newShooter.stop();
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