package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Turret - BioBuzz Blue", group = "Turret")
public class TurretAutoAlignBlueOpMode extends OpMode {

    private Limelight3A limelight;
    private final TurretMechanism turret = new TurretMechanism();

    // Blue alliance tag clusters
    private static final int[] CLUSTER_1 = {38, 39, 40, 41};
    private static final int[] CLUSTER_2 = {42, 43, 44, 45};

    @Override
    public void init() {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        turret.init(hardwareMap);

        // Use your AprilTag pipeline
        limelight.pipelineSwitch(0);

        telemetry.addLine("Blue BioBuzz Turret");
        telemetry.addLine("Initialized");
        telemetry.update();
    }

    @Override
    public void start() {

        limelight.start();
        turret.resetTimer();
    }

    @Override
    public void loop() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {

            turret.stop();

            telemetry.addLine("No valid Limelight result");
            telemetry.update();

            return;
        }

        List<FiducialResult> fiducials =
                result.getFiducialResults();

        // Find the two clusters
        List<FiducialResult> cluster1 =
                getCluster(fiducials, CLUSTER_1);

        List<FiducialResult> cluster2 =
                getCluster(fiducials, CLUSTER_2);

        // Determine which cluster is higher
        List<FiducialResult> targetCluster =
                chooseHigherCluster(cluster1, cluster2);

        if (targetCluster == null || targetCluster.isEmpty()) {

            turret.stop();

            telemetry.addLine("No Blue BioBuzz tags detected");

            telemetry.update();
            return;
        }

        // Pick the tag in the selected cluster
        // that is closest to the center of the camera.
        FiducialResult target =
                getBestTarget(targetCluster);

        if (target == null) {

            turret.stop();
            return;
        }

        double tx = target.getTargetXDegrees();

        // Aim turret
        turret.updateAuto(tx);

        // ---------------- TELEMETRY ----------------

        telemetry.addData(
                "Target ID",
                target.getFiducialId()
        );

        telemetry.addData(
                "TX",
                "%.2f°",
                tx
        );

        telemetry.addData(
                "TY",
                "%.2f°",
                target.getTargetYDegrees()
        );

        telemetry.addData(
                "Cluster 1 Avg TY",
                "%.2f°",
                averageTY(cluster1)
        );

        telemetry.addData(
                "Cluster 2 Avg TY",
                "%.2f°",
                averageTY(cluster2)
        );

        telemetry.addData(
                "Selected Cluster",
                targetCluster == cluster1 ? "38-41" : "42-45"
        );

        telemetry.update();
    }

    /**
     * Returns all visible tags belonging to a particular cluster.
     */
    private List<FiducialResult> getCluster(
            List<FiducialResult> allTags,
            int[] clusterIDs) {

        List<FiducialResult> result =
                new ArrayList<>();

        for (FiducialResult tag : allTags) {

            int id = tag.getFiducialId();

            for (int clusterID : clusterIDs) {

                if (id == clusterID) {
                    result.add(tag);
                    break;
                }
            }
        }

        return result;
    }

    /**
     * Determines which cluster is physically higher
     * based on the average TY of its visible tags.
     */
    private List<FiducialResult> chooseHigherCluster(
            List<FiducialResult> cluster1,
            List<FiducialResult> cluster2) {

        if (cluster1.isEmpty() && cluster2.isEmpty()) {
            return null;
        }

        if (cluster1.isEmpty()) {
            return cluster2;
        }

        if (cluster2.isEmpty()) {
            return cluster1;
        }

        double ty1 = averageTY(cluster1);
        double ty2 = averageTY(cluster2);

        if (ty1 > ty2) {
            return cluster1;
        } else {
            return cluster2;
        }
    }

    /**
     * Average vertical angle of the visible tags.
     */
    private double averageTY(
            List<FiducialResult> cluster) {

        if (cluster.isEmpty()) {
            return Double.NaN;
        }

        double total = 0;

        for (FiducialResult tag : cluster) {
            total += tag.getTargetYDegrees();
        }

        return total / cluster.size();
    }

    /**
     * Pick the tag closest to the camera's center.
     */
    private FiducialResult getBestTarget(
            List<FiducialResult> cluster) {

        FiducialResult best = null;
        double bestAbsTX = Double.MAX_VALUE;

        for (FiducialResult tag : cluster) {

            double tx =
                    Math.abs(tag.getTargetXDegrees());

            if (tx < bestAbsTX) {
                bestAbsTX = tx;
                best = tag;
            }
        }

        return best;
    }

    @Override
    public void stop() {

        turret.stop();
        limelight.stop();
    }
}
