package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BlueHiveTipDetector {

    // BioBuzz BLUE CELL AprilTags
    // 38-41 = side opposite audience
    // 42-45 = audience side
    private static final int[] BLUE_HIVE_TAGS = {
            38, 39, 40, 41,
            42, 43, 44, 45
    };

    private Limelight3A limelight;

    public void init(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Set this to your AprilTag pipeline number
        limelight.pipelineSwitch(0);

        limelight.start();
    }

    /**
     * Returns true if any BLUE hive AprilTag is currently visible.
     */
    public boolean areHiveTagsVisible() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            for (int hiveTag : BLUE_HIVE_TAGS) {

                if (id == hiveTag) {
                    return true;
                }
            }
        }

        return false;
    }

    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }
}