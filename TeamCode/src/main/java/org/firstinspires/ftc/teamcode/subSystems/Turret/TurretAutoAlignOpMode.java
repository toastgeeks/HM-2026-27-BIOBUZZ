package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import java.util.List;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretAutoAlignOpMode extends OpMode {
    private Limelight3A limelight;
    private TurretMechanism turret = new TurretMechanism();

    // --------------used to auto update P and D ---------------
    double[] stepSizes = {0.1, 0.01, 0.001, 0.0001, 0.00001};
    //Index to select the current step size from the array
    int stepIndex = 2;

    @Override
    public void init() {

        //get limelight
        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        // Initialize turret
        turret.init(hardwareMap);

        //select pipeline
        limelight.pipelineSwitch(0);

        telemetry.addLine("Initialized");

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

        LLResult result = limelight.getLatestResult();

        double tx = Double.NaN;
        int detectedID = -1;

        // look for april tag 20

        if (result != null && result.isValid()) {

            List<FiducialResult> fiducials =
                    result.getFiducialResults();

            for (FiducialResult fiducial : fiducials) {

                if (fiducial.getFiducialId() == 20) {

                    tx = fiducial.getTargetXDegrees();
                    detectedID = 20;

                    break;
                }
            }
        }

        turret.update(tx);

        // update P and D on the fly
        // 'B' button cycles through the different step sizes for tuning precision.
        if (gamepad2.bWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length; // Modulo wraps the index back to 0.
        }

        // D-pad left/right adjusts the P gain.
        if (gamepad2.dpadLeftWasPressed()) {
            turret.setkP(turret.getkP() - stepSizes[stepIndex]);
        }
        if (gamepad2.dpadRightWasPressed()) {
            turret.setkP(turret.getkP() + stepSizes[stepIndex]);
        }

        // D-pad up/down adjusts the D gain.
        if (gamepad2.dpadUpWasPressed()) {
            turret.setkD(turret.getkD() - stepSizes[stepIndex]);
        }

        if (gamepad2.dpadDownWasPressed()) {
            turret.setkD(turret.getkD() + stepSizes[stepIndex]);
        }


        if (detectedID == 20) {

            telemetry.addData("Tag", "20");
            telemetry.addData("TX", "%.2f°", tx);

        } else {

            telemetry.addLine("No Tag 20 Detected");
        }
        telemetry.addLine("---------------------------");
        telemetry.addData("Tuning P", "%.5f (D-Pad L/R)", turret.getkP());
        telemetry.addData("Tuning D", "%.5f (D-Pad U/D)", turret.getkD());
        telemetry.addData("Step Size", "%.5f (B Button)", stepSizes[stepIndex]);

    }

}
