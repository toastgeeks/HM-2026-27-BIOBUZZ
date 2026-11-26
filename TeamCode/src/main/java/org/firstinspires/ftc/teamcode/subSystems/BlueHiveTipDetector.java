package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class BlueHiveTipDetector {

    private Limelight3A limelight;

    // BioBuzz BLUE hive/CELL AprilTags
    // 38-41 = opposite audience side
    // 42-45 = audience side
    private static final int[] BLUE_HIVE_TAGS = {
            38, 39, 40, 41,
            42, 43, 44, 45
    };

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // CHANGE THIS if your AprilTag pipeline isn't pipeline 0
        limelight.pipelineSwitch(0);

        limelight.start();
    }

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