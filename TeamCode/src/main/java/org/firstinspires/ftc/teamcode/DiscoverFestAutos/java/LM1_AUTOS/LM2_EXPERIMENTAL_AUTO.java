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

import com.skeletonarmy.marrow.prompts.*;
import com.skeletonarmy.marrow.weaver.*;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.subSystems.*;

import java.util.ArrayList;
import java.util.List;

import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.*;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

@Autonomous(name = "LM2 Auto")
public class LM2_EXPERIMENTAL_AUTO extends OpMode {

    private IntakeCode intake;
    private NEW_SHOOTER_TRANSFER newShooter;
    private Follower follower;
    private FlowerIntake arm;
    private RedHiveTipDetector hiveDetector;

    // =========================================================
    // PRE-START OPTIONS
    // =========================================================

    private enum AutoMode {
        EXISTING_AUTO,
        CUSTOM_AUTO
    }

    private enum Alliance {
        RED,
        BLUE
    }

    private enum StartLocation {
        CLOSE,
        FAR
    }

    private enum ParkLocation {
        LEFT,
        RIGHT
    }

    private enum Task {
        NONE,
        SHOOT_CLOSE,
        SHOOT_FAR,
        INTAKE_FLOWER_1,
        INTAKE_FLOWER_2,
        PARK_LEFT,
        PARK_RIGHT
    }

    private Prompter prompter;

    private AutoMode selectedMode = AutoMode.EXISTING_AUTO;
    private Alliance selectedAlliance = Alliance.RED;
    private StartLocation selectedStart = StartLocation.CLOSE;
    private ParkLocation selectedPark = ParkLocation.LEFT;

    private Task task1 = Task.NONE;
    private Task task2 = Task.NONE;
    private Task task3 = Task.NONE;
    private Task task4 = Task.NONE;
    private Task task5 = Task.NONE;
    private Task task6 = Task.NONE;

    private double delaySeconds = 0.0;
    private boolean promptsComplete = false;

    private void setupPrompter() {
        prompter = new Prompter(this);

        prompter.prompt(
                        "mode",
                        new OptionPrompt<>("Select Autonomous",
                                AutoMode.EXISTING_AUTO,
                                AutoMode.CUSTOM_AUTO)
                ).label("Auto Type")

                .prompt(
                        "alliance",
                        new OptionPrompt<>("Select Alliance",
                                Alliance.RED,
                                Alliance.BLUE)
                ).label("Alliance")

                .prompt(
                        "start",
                        new OptionPrompt<>("Custom Auto Start",
                                StartLocation.CLOSE,
                                StartLocation.FAR)
                ).label("Custom Start Position")

                .prompt(
                        "delay",
                        new ValuePrompt<>("Start Delay (seconds)",
                                Double.class,
                                0.0,
                                20.0,
                                0.0,
                                1.0)
                ).label("Start Delay")

                .prompt(
                        "park",
                        new OptionPrompt<>("Default Auto Parking Position",
                                ParkLocation.LEFT,
                                ParkLocation.RIGHT)
                ).label("Default Auto Parking")

                .prompt(
                        "task1",
                        new OptionPrompt<>("Custom Task 1",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 1")

                .prompt(
                        "task2",
                        new OptionPrompt<>("Custom Task 2",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 2")

                .prompt(
                        "task3",
                        new OptionPrompt<>("Custom Task 3",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 3")

                .prompt(
                        "task4",
                        new OptionPrompt<>("Custom Task 4",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 4")

                .prompt(
                        "task5",
                        new OptionPrompt<>("Custom Task 5",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 5")

                .prompt(
                        "task6",
                        new OptionPrompt<>("Custom Task 6",
                                Task.NONE, Task.SHOOT_CLOSE, Task.SHOOT_FAR,
                                Task.INTAKE_FLOWER_1, Task.INTAKE_FLOWER_2,
                                Task.PARK_LEFT, Task.PARK_RIGHT)
                ).label("Task 6")

                .showSummary()
                .onComplete(() -> {
                    selectedMode = prompter.get("mode");
                    selectedAlliance = prompter.get("alliance");
                    selectedStart = prompter.get("start");
                    delaySeconds = prompter.get("delay");
                    selectedPark = prompter.get("park");

                    task1 = prompter.get("task1");
                    task2 = prompter.get("task2");
                    task3 = prompter.get("task3");
                    task4 = prompter.get("task4");
                    task5 = prompter.get("task5");
                    task6 = prompter.get("task6");

                    promptsComplete = true;
                });
    }

    // =========================================================
    // POSES
    // =========================================================

    private final PoseFactory poseFactory = PoseFactory.degrees();

    // Existing close-side start pose.
    private final Pose startPose = poseFactory.of(55.5, 8.4, 270);

    // Corrected far-side start pose supplied by the team.
    private final Pose farStartPose = poseFactory.of(62.75, 132.75, 90);

    // Close shooting position is the same as the close start.
    private Pose getCloseShootPose() {
        return startPose;
    }

    // Far shooting position is the corrected far start position.
    private Pose getFarShootPose() {
        return farStartPose;
    }

    private final Pose goToIntakeFromFlower1 =
            poseFactory.of(23.1224, 46.518, 180);

    private final Pose goToIntakeFromFlower1Control1 =
            poseFactory.of(57.2165, 24.522, 0);

    private final Pose intakeFromFlower1 =
            poseFactory.of(14.5228, 46.5714, 180);

    private final Pose backUpFromFlower1 =
            poseFactory.of(22.8507, 46.7545, 180);

    private final Pose goShoot1Start =
            poseFactory.of(22.8507, 46.7545, 180);

    private final Pose goShoot1 =
            poseFactory.of(60.9503, 130.6471, 90);

    private final Pose goShoot1Control1 =
            poseFactory.of(21.7767, 115.867, 0);

    private final Pose goShoot1Control2 =
            poseFactory.of(62.261, 114.3159, 0);

    private final Pose goToIntakeFromFlower2 =
            poseFactory.of(47.2125, 113.7953, 90);

    private final Pose goToIntakeFromFlower2Control1 =
            poseFactory.of(55.4944, 105.8768, 0);

    private final Pose intakeFromFlower2 =
            poseFactory.of(47.2125, 127.8455, 90.0001);

    private final Pose backUpFromFlower2 =
            poseFactory.of(47.0293, 113.5205, 90);

    private final Pose goShoot2Start =
            poseFactory.of(47.0293, 113.5205, 90);

    private final Pose goShoot2 =
            poseFactory.of(61.3167, 130.006, 90);

    private final Pose goShoot2Control1 =
            poseFactory.of(55.443, 104.9952, 0);

    private final Pose goParkStart =
            poseFactory.of(61.3167, 130.006, 90);

    private final Pose goParkControl1 =
            poseFactory.of(56.7729, 110.0926, 0);

    private Pose getParkPose() {
        if (selectedPark == ParkLocation.LEFT) {
            return poseFactory.of(13.78, 115.9, 0);
        } else {
            return poseFactory.of(13.78, 98.0, 0);
        }
    }

    private Pose getParkPose(ParkLocation location) {
        if (location == ParkLocation.LEFT) {
            return poseFactory.of(13.78, 115.9, 0);
        } else {
            return poseFactory.of(13.78, 98.0, 0);
        }
    }

    // =========================================================
    // WEAVER / HIVE OBSTACLE
    // =========================================================

    /*
     * Hive frame corners supplied by the team:
     * X = 46.11 to 95.94
     * Y = 51.33 to 91.27
     *
     * Points are ordered around the perimeter of the rectangle.
     */
    private final PolygonZone hiveObstacle = new PolygonZone(
            new Point(46.11, 51.33),
            new Point(95.94, 51.33),
            new Point(95.94, 91.27),
            new Point(46.11, 91.27)
    );

    /*
     * Initial robot footprint estimate. Change these to the actual
     * bumper-to-bumper dimensions if the robot is larger or smaller.
     * Clearance is extra space beyond the robot footprint.
     */
    private static final double ROBOT_LENGTH_IN = 18.0;
    private static final double ROBOT_WIDTH_IN = 18.0;
    private static final double HIVE_CLEARANCE_IN = 4.0;

    private void configureWeaver() {
        Weaver.setConfig(
                new PathConfig()
                        .robotWidth(ROBOT_LENGTH_IN)
                        .robotHeight(ROBOT_WIDTH_IN)
                        .clearance(HIVE_CLEARANCE_IN)
                        .smoothing(true)
        );
    }

    private Path generateWeaverPath(Pose from, Pose to) {
        PathResult result = Weaver.builder()
                .start(new Point(from.x(), from.y(), from.heading()))
                .end(new Point(to.x(), to.y(), to.heading()), HeadingPolicy.FREE)
                .addObstacle(hiveObstacle)
                .generate();

        if (result.getSegments() == null || result.getSegments().isEmpty()) {
            throw new IllegalStateException("Weaver did not generate a usable path.");
        }

        if (!result.getUnvisitedTargets().isEmpty()) {
            throw new IllegalStateException(
                    "Weaver could not route to every requested target: "
                            + result.getUnvisitedTargets()
            );
        }

        return PedroPathingConverter.toPath(result);
    }

    private boolean samePose(Pose a, Pose b, double toleranceInches) {
        if (a == null || b == null) return false;

        double dx = a.x() - b.x();
        double dy = a.y() - b.y();

        return Math.hypot(dx, dy) <= toleranceInches;
    }

    /*
     * Use a manually tuned path when the robot is at that path's intended
     * starting point. Otherwise, generate a connection with Weaver.
     */
    private Command goToPose(
            Pose destination,
            Path preferredManualPath,
            Pose preferredManualStart
    ) {
        return lazy(() -> {
            Pose current = follower.pose();

            if (samePose(current, destination, 1.0)) {
                return instant(() -> { });
            }

            if (preferredManualPath != null
                    && preferredManualStart != null
                    && samePose(current, preferredManualStart, 2.0)) {
                return follow(follower, preferredManualPath);
            }

            Path generated = generateWeaverPath(current, destination);
            return follow(follower, generated);
        });
    }

    private Command goToPose(Pose destination) {
        return goToPose(destination, null, null);
    }

    // =========================================================
    // EXISTING MANUAL PATHS - PRESERVED
    // =========================================================

    public Path goToIntakeFromFlower1() {
        return Paths.curve(
                startPose,
                goToIntakeFromFlower1Control1,
                goToIntakeFromFlower1
        ).linear(startPose, goToIntakeFromFlower1);
    }

    public Path intakeFromFlower1() {
        return Paths.line(
                goToIntakeFromFlower1,
                intakeFromFlower1
        ).constant(intakeFromFlower1);
    }

    public Path backUpFromFlower1() {
        return Paths.line(
                intakeFromFlower1,
                backUpFromFlower1
        ).constant(backUpFromFlower1);
    }

    public Path goShoot1() {
        return Paths.curve(
                goShoot1Start,
                goShoot1Control1,
                goShoot1Control2,
                goShoot1
        ).linear(goShoot1Start, goShoot1);
    }

    public Path goToIntakeFromFlower2() {
        return Paths.curve(
                goShoot1,
                goToIntakeFromFlower2Control1,
                goToIntakeFromFlower2
        ).constant(goToIntakeFromFlower2);
    }

    public Path intakeFromFlower2() {
        return Paths.line(
                goToIntakeFromFlower2,
                intakeFromFlower2
        ).tangent();
    }

    public Path backUpFromFlower2() {
        return Paths.line(
                intakeFromFlower2,
                backUpFromFlower2
        ).constant(backUpFromFlower2);
    }

    public Path goShoot2() {
        return Paths.curve(
                goShoot2Start,
                goShoot2Control1,
                goShoot2
        ).linear(goShoot2Start, goShoot2);
    }

    public Path goPark() {
        return Paths.curve(
                goParkStart,
                goParkControl1,
                getParkPose()
        ).linear(goParkStart, getParkPose());
    }

    // =========================================================
    // TIMEOUT
    // =========================================================

    private Command followWithTimeout(Path path, long timeoutMs) {
        return lazy(() -> {
            long startTime = System.currentTimeMillis();

            return sequential(
                    follow(follower, path).until(
                            () -> System.currentTimeMillis() - startTime >= timeoutMs
                    ),
                    instant(() -> {
                        if (System.currentTimeMillis() - startTime >= timeoutMs) {
                            follower.stop();
                        }
                    })
            );
        });
    }

    // =========================================================
    // EXISTING AUTO - PRESERVED
    // =========================================================

    private Command autoRoutine() {
        return sequential(
                waitMs((long) (delaySeconds * 1000)),

                newShooter.rev(),
                waitMs(1000),
                newShooter.fire(2500),

                intake.run(1.0),

                follow(follower, goToIntakeFromFlower1()),

                arm.lowerArm(),
                waitMs(500),

                followWithTimeout(intakeFromFlower1(), 3000),

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

                followWithTimeout(intakeFromFlower2(), 3000),

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

    // =========================================================
    // CUSTOM TASK COMMANDS
    // =========================================================

    private Command shootAt(Pose shootingPose) {
        return sequential(
                goToPose(shootingPose),
                newShooter.rev(),
                waitMs(1000),
                newShooter.fire(2500)
        );
    }

    private Command intakeFlower1Task() {
        return sequential(
                intake.run(1.0),

                goToPose(
                        goToIntakeFromFlower1,
                        goToIntakeFromFlower1(),
                        startPose
                ),

                arm.lowerArm(),
                waitMs(500),

                followWithTimeout(intakeFromFlower1(), 3000),
                waitMs(1500),

                follow(follower, backUpFromFlower1()),

                arm.raiseArm()
        );
    }

    private Command intakeFlower2Task() {
        return sequential(
                intake.run(1.0),

                goToPose(
                        goToIntakeFromFlower2,
                        goToIntakeFromFlower2(),
                        goShoot1
                ),

                arm.lowerArm(),
                waitMs(500),

                followWithTimeout(intakeFromFlower2(), 3000),
                waitMs(1500),

                follow(follower, backUpFromFlower2()),

                arm.raiseArm()
        );
    }

    private Command parkTask(ParkLocation location) {
        Pose destination = getParkPose(location);

        Path manualParkPath = Paths.curve(
                goParkStart,
                goParkControl1,
                destination
        ).linear(goParkStart, destination);

        return sequential(
                goToPose(destination, manualParkPath, goParkStart),
                newShooter.stopRev(),
                intake.run(0.0)
        );
    }

    private Command commandForTask(Task task) {
        switch (task) {
            case SHOOT_CLOSE:
                return shootAt(getCloseShootPose());

            case SHOOT_FAR:
                return shootAt(getFarShootPose());

            case INTAKE_FLOWER_1:
                return intakeFlower1Task();

            case INTAKE_FLOWER_2:
                return intakeFlower2Task();

            case PARK_LEFT:
                return parkTask(ParkLocation.LEFT);

            case PARK_RIGHT:
                return parkTask(ParkLocation.RIGHT);

            case NONE:
            default:
                return instant(() -> { });
        }
    }

    private Command customAutoRoutine() {
        List<Command> commands = new ArrayList<>();

        commands.add(waitMs((long) (delaySeconds * 1000)));

        Task[] tasks = {
                task1, task2, task3, task4, task5, task6
        };

        for (Task task : tasks) {
            if (task != null && task != Task.NONE) {
                commands.add(commandForTask(task));
            }
        }

        return sequential(commands.toArray(new Command[0]));
    }

    // =========================================================
    // INIT
    // =========================================================

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

        newShooter.closeGate().schedule();
        arm.raiseArm().schedule();
        newShooter.stop();

        configureWeaver();
        setupPrompter();

        Scheduler.execute();
    }

    @Override
    public void init_loop() {
        prompter.run();
    }

    // =========================================================
    // START
    // =========================================================

    @Override
    public void start() {
        if (!promptsComplete) {
            telemetry.addLine("Prompts incomplete - using defaults");
            telemetry.update();
        }

        if (selectedMode == AutoMode.CUSTOM_AUTO) {
            Pose selectedStartPose = selectedStart == StartLocation.FAR
                    ? farStartPose
                    : startPose;

            follower.setPose(selectedStartPose);
            schedule(customAutoRoutine());
        } else {
            // The existing hand-tuned auto was designed for the close start.
            follower.setPose(startPose);
            schedule(autoRoutine());
        }
    }

    // =========================================================
    // LOOP
    // =========================================================

    @Override
    public void loop() {
        follower.update();
        Scheduler.execute();

        telemetry.addData("X", follower.pose().x());
        telemetry.addData("Y", follower.pose().y());
        telemetry.addData(
                "Heading",
                Math.toDegrees(follower.pose().heading())
        );

        telemetry.addData("Follower Mode", follower.mode());
        telemetry.addData("Auto Mode", selectedMode);
        telemetry.addData("Alliance", selectedAlliance);
        telemetry.addData("Start", selectedStart);
        telemetry.addData("Start Delay", delaySeconds);
        telemetry.addData("Default Parking", selectedPark);

        telemetry.addData("Task 1", task1);
        telemetry.addData("Task 2", task2);
        telemetry.addData("Task 3", task3);
        telemetry.addData("Task 4", task4);
        telemetry.addData("Task 5", task5);
        telemetry.addData("Task 6", task6);

        telemetry.update();
    }

    // =========================================================
    // STOP
    // =========================================================

    @Override
    public void stop() {
        if (follower != null) {
            OpModeStorage.autonomousEndPose = follower.pose();
        }

        if (newShooter != null) {
            newShooter.stop();
        }
    }
}
