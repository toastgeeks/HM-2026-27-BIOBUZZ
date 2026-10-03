package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class RedHiveTipDetector {

    private Limelight3A limelight;

    // BioBuzz RED CELL AprilTags
    // 30-33 = side opposite audience
    // 34-37 = audience side
    private static final int[] RED_HIVE_TAGS = {
            30, 31, 32, 33,
            34, 35, 36, 37
    };

    public void init(HardwareMap hardwareMap) {

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Set this to your AprilTag pipeline number
        limelight.pipelineSwitch(0);

        limelight.start();
    }


     // Returns true if any RED hive AprilTag is currently visible.

    public boolean areHiveTagsVisible() {

        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return false;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            for (int hiveTag : RED_HIVE_TAGS) {

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