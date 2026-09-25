package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import org.firstinspires.ftc.teamcode.subSystems.OpModeStorage;
import org.firstinspires.ftc.teamcode.subSystems.Shooter_Transfer;
import org.firstinspires.ftc.teamcode.subSystems.IntakeCode;
import com.pedropathing.ivy.Command;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.commands.Commands.*;


@Autonomous
public class RED_BASKET_START extends OpMode {

    private IntakeCode intake;
    private Shooter_Transfer shooter;
    private Follower follower;
    private final PoseFactory p = PoseFactory.degrees();
    private final Pose startPose = p.of(55.5, 8.4, 90);
    private final Pose goToGarden = p.of(19.6, 7.9, 180);
    private final Pose goToGardenControl = p.of(47.92198581560284, 22.442080378250598, 180);
    private final Pose intakeFromGarden = p.of(8.684397163120554, 8.189125295508282, 180);
    private final Pose goShootOtherSide = p.of(59.02127659574469, 132.5579196217494, 270);
    private final Pose goShootOtherSideControl1 = p.of(75.46217494089834, 8.569739952718681, 270);
    private final Pose goShootOtherSideControl2 = p.of(56.70035460992909, 37.40648564548357, 270);
    private final Pose backUpForIntake = p.of(59.21040189125295, 114.87115839243496, 90);
    private final Pose farIntake = p.of(58.495271867612296, 132.57565011820333, 90);
    private final Pose goPark = p.of(13.34633569739953, 113.07801418439716, 0);
    private final Pose goParkControl = p.of(53.04964539007092, 110.89952718676122, 0);




    private Path goIntake() {
        return curve(startPose, goToGardenControl, goToGarden).linear(startPose, goToGarden);
    }
    private Path closeIntake(){
        return line(goToGarden,intakeFromGarden).linear(goToGarden,intakeFromGarden);
    }
    private Path goShootFar() {
        return curve(intakeFromGarden,goShootOtherSideControl1,goShootOtherSideControl2,goShootOtherSide).linear(intakeFromGarden, goShootOtherSide);
    }
    private Path backUpForIntake(){
        return line(goShootOtherSide,backUpForIntake).linear(goShootOtherSide,backUpForIntake);
    }
    private Path farIntake(){
        return line(backUpForIntake,farIntake).linear(backUpForIntake,farIntake);
    }
    private Path goPark(){
        return curve(farIntake,goParkControl,goPark).linear(farIntake,goPark);
    }

    private Command autoRoutine() {
        double time_between_shots = 250;
        double flywheel_spinup_time = 800;
        return sequential(

                instant(() -> Shooter_Transfer.shooterState = 1),

                waitMs(3000),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 0),

                instant(() -> Shooter_Transfer.shooterState = 0),

                instant(() -> intake.setIntakeSpeed(1.0)),

                follow(follower, goIntake()),

                follow(follower, closeIntake()),

                instant(() -> intake.setIntakeSpeed(0.0)),

                instant(() -> Shooter_Transfer.shooterState = 1),

                follow(follower, goShootFar()),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                waitMs(flywheel_spinup_time),

                instant(() -> Shooter_Transfer.servoState = 1),

                waitMs(time_between_shots),

                instant(() -> Shooter_Transfer.servoState = 0),

                instant(() -> Shooter_Transfer.shooterState = 0),

                follow(follower, backUpForIntake()),

                instant(() -> intake.setIntakeSpeed(1.0)),

                follow(follower, farIntake()),

                instant(() -> intake.setIntakeSpeed(0.0)),

                follow(follower, goPark())
        );
    }


    @Override
    public void init() {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);

        intake = new IntakeCode();
        intake.init(hardwareMap);

        shooter = new Shooter_Transfer();
        shooter.init(hardwareMap);

        Shooter_Transfer.servoState = 0;
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
