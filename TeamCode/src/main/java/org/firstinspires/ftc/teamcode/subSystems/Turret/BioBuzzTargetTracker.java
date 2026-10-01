package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;

import java.util.List;

public class BioBuzzTargetTracker {

    public enum Alliance {
        RED,
        BLUE
    }

    private Alliance alliance;

    public BioBuzzTargetTracker(Alliance alliance) {
        this.alliance = alliance;
    }

    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * Returns the AprilTag we want to aim at.
     */
    public FiducialResult findTarget(List<FiducialResult> tags) {

        FiducialResult bestTarget = null;

        // Find the two alliance-specific clusters.
        FiducialResult cluster1 = null;
        FiducialResult cluster2 = null;

        for (FiducialResult tag : tags) {

            int id = tag.getFiducialId();

            if (!isAllianceTag(id)) {
                continue;
            }

            if (isFirstCluster(id)) {

                if (cluster1 == null ||
                        tag.getTargetYDegrees() >
                                cluster1.getTargetYDegrees()) {

                    cluster1 = tag;
                }

            } else if (isSecondCluster(id)) {

                if (cluster2 == null ||
                        tag.getTargetYDegrees() >
                                cluster2.getTargetYDegrees()) {

                    cluster2 = tag;
                }
            }
        }

        // Nothing visible
        if (cluster1 == null && cluster2 == null) {
            return null;
        }

        // Only cluster 1 visible
        if (cluster2 == null) {
            return cluster1;
        }

        // Only cluster 2 visible
        if (cluster1 == null) {
            return cluster2;
        }

        // Both visible.
        // Higher TY = physically higher in Limelight's view.
        if (cluster1.getTargetYDegrees() >
                cluster2.getTargetYDegrees()) {

            bestTarget = cluster1;

        } else {

            bestTarget = cluster2;
        }

        return bestTarget;
    }

    private boolean isAllianceTag(int id) {

        if (alliance == Alliance.RED) {
            return id >= 30 && id <= 37;
        }

        return id >= 38 && id <= 45;
    }

    /**
     * First cluster:
     *
     * RED: 30-33
     * BLUE: 38-41
     */
    private boolean isFirstCluster(int id) {

        if (alliance == Alliance.RED) {
            return id >= 30 && id <= 33;
        }

        return id >= 38 && id <= 41;
    }

    /**
     * Second cluster:
     *
     * RED: 34-37
     * BLUE: 42-45
     */
    private boolean isSecondCluster(int id) {

        if (alliance == Alliance.RED) {
            return id >= 34 && id <= 37;
        }

        return id >= 42 && id <= 45;
    }
}
