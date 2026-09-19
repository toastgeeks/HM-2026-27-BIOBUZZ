package org.firstinspires.ftc.teamcode.DiscoverFestAutos.java;




import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.pedropathing.follower.Follower;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.robocol.TelemetryMessage;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.paths.Path;
import com.pedropathing.ivy.Scheduler;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import static com.pedropathing.ivy.Scheduler.schedule;


@Autonomous
public class RED_RIGHT extends OpMode {
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




    @Override
    public void init() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(startPose);
    }
    @Override
    public void start() {
        schedule(follow(follower, goIntake()));
        schedule(follow(follower, closeIntake()));
        schedule(follow(follower, goShootFar()));
        schedule(follow(follower, backUpForIntake()));
        schedule(follow(follower, farIntake()));
        schedule(follow(follower, goPark()));
        //we're done!!!
    }


    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();




        TelemetryMessage telemetryData = null;
        telemetryData.addData("X", follower.pose().x());
        telemetryData.addData("Y", follower.pose().y());
        telemetryData.addData("Heading", Math.toDegrees(follower.pose().heading()));
        telemetry.addData("Follower Mode", follower.mode());
        telemetry.update();
    }
}
