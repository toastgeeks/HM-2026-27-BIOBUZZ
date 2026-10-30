package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java.LM1_AUTOS;

import com.pedropathing.api.Paths;
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

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "LM2 Auto")
public class LM2_RED_AUTO extends OpMode {

    private IntakeCode intake;
    private NEW_SHOOTER_TRANSFER newShooter;
    private Follower follower;
    private FlowerIntake arm;
    private RedHiveTipDetector hiveDetector;

    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose startPose = poseFactory.of(55.5, 8.4, 270);
    private final Pose goToIntakeFromFlower1 = poseFactory.of(23.1224, 46.518, 180);
    private final Pose goToIntakeFromFlower1Control1 = poseFactory.of(57.2165, 24.522, 0);
    private final Pose intakeFromFlower1 = poseFactory.of(14.5228, 46.5714, 180);
    private final Pose backUpFromFlower1 = poseFactory.of(22.8507, 46.7545, 180);
    private final Pose goShoot1Start = poseFactory.of(22.8507, 46.7545, 180);
    private final Pose goShoot1 = poseFactory.of(60.9503, 130.6471, 90);
    private final Pose goShoot1Control1 = poseFactory.of(21.7767, 115.867, 0);
    private final Pose goShoot1Control2 = poseFactory.of(62.261, 114.3159, 0);
    private final Pose goToIntakeFromFlower2 = poseFactory.of(47.2125, 113.7953, 90);
    private final Pose goToIntakeFromFlower2Control1 = poseFactory.of(55.4944, 105.8768, 0);
    private final Pose intakeFromFlower2 = poseFactory.of(47.2125, 127.8455, 90.0001);
    private final Pose backUpFromFlower2 = poseFactory.of(47.0293, 113.5205, 90);
    private final Pose goShoot2Start = poseFactory.of(47.0293, 113.5205, 90);
    private final Pose goShoot2 = poseFactory.of(61.3167, 130.006, 90);
    private final Pose goShoot2Control1 = poseFactory.of(55.443, 104.9952, 0);
    private final Pose goParkStart = poseFactory.of(61.3167, 130.006, 90);
    private final Pose goPark = poseFactory.of(15, 115.9018, 0);
    private final Pose goParkControl1 = poseFactory.of(56.7729, 110.0926, 0);

    public Path goToIntakeFromFlower1() {
        return Paths.curve(startPose, goToIntakeFromFlower1Control1, goToIntakeFromFlower1).linear(startPose, goToIntakeFromFlower1);
    }

    public Path intakeFromFlower1() {
        return Paths.line(goToIntakeFromFlower1, intakeFromFlower1).constant(intakeFromFlower1);
    }

    public Path backUpFromFlower1() {
        return Paths.line(intakeFromFlower1, backUpFromFlower1).constant(backUpFromFlower1);
    }

    public Path goShoot1() {
        return Paths.curve(goShoot1Start, goShoot1Control1, goShoot1Control2, goShoot1).linear(goShoot1Start, goShoot1);
    }

    public Path goToIntakeFromFlower2() {
        return Paths.curve(goShoot1, goToIntakeFromFlower2Control1, goToIntakeFromFlower2).constant(goToIntakeFromFlower2);
    }

    public Path intakeFromFlower2() {
        return Paths.line(goToIntakeFromFlower2, intakeFromFlower2).tangent();
    }

    public Path backUpFromFlower2() {
        return Paths.line(intakeFromFlower2, backUpFromFlower2).constant(backUpFromFlower2);
    }

    public Path goShoot2() {
        return Paths.curve(goShoot2Start, goShoot2Control1, goShoot2).linear(goShoot2Start, goShoot2);
    }

    public Path goPark() {
        return Paths.curve(goParkStart, goParkControl1, goPark).linear(goParkStart, goPark);
    }

    private Command autoRoutine() {
        return sequential(
                newShooter.rev(),
                waitMs(1000),
                newShooter.fire(2500),
                intake.run(1.0),
                follow(follower, goToIntakeFromFlower1()),
                arm.lowerArm(),
                waitMs(500),
                follow(follower, intakeFromFlower1()),
                waitMs(1500),
                follow(follower, backUpFromFlower1()),
                arm.raiseArm(),
                newShooter.rev(),
                follow(follower, goShoot1()),
                waitMs(500),
                newShooter.fire(2500),
                follow(follower, goToIntakeFromFlower2()),
                arm.lowerArm(),
                waitMs(500),
                follow(follower, intakeFromFlower2()),
                waitMs(1500),
                follow(follower, backUpFromFlower2()),
                arm.raiseArm(),
                newShooter.rev(),
                follow(follower, goShoot2()),
                newShooter.fire(2500),
                newShooter.stopRev(),
                follow(follower, goPark()),
                intake.run(0.0)
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