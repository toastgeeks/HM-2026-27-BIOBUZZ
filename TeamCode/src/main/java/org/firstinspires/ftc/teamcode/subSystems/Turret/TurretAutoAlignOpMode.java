package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.openftc.apriltag.AprilTagDetection;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

public class TurretAutoAlignOpMode extends OpMode {
    Limelight3A limelight;
    private TurretMechanism turret = new TurretMechanism();

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        turret.init(hardwareMap);
        limelight.pipelineSwitch(0);

        telemetry.addLine("Initialized all mechanisms");
    }

    public void start() {
        turret.resetTimer();
        limelight.start();
    }

    @Override
    public void loop() {
        // vision logic
        AprilTagDetection id20 = fiducial.getFiducialId(20);

    }

}
