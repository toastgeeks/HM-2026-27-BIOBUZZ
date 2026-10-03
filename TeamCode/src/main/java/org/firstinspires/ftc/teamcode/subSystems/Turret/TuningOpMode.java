package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

import java.util.List;

@TeleOp(name = "Turret - Tuning", group = "Turret")
public class TuningOpMode extends OpMode {

    private Limelight3A limelight;
    private TelemetryManager panelsTelemetry;

    private final TurretMechanism turret =
            new TurretMechanism();


    // ---------------- P/D TUNING ----------------

    double[] stepSizes = {
            0.1,
            0.01,
            0.001,
            0.0001,
            0.00001
    };

    int stepIndex = 2;


    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        turret.init(hardwareMap);

        // AprilTag pipeline
        limelight.pipelineSwitch(0);

        panelsTelemetry.addLine(
                "Turret Tuning Mode"
        );

        panelsTelemetry.addLine(
                "Target: AprilTag 20"
        );

        panelsTelemetry.update();
    }


    @Override
    public void start() {

        limelight.start();

        turret.resetTimer();
    }


    @Override
    public void loop() {

        // GET LIMELIGHT RESULT

        LLResult result =
                limelight.getLatestResult();

        double tx = Double.NaN;

        FiducialResult target = null;

        // FIND TAG 24

        if (result != null && result.isValid()) {

            List<FiducialResult> fiducials =
                    result.getFiducialResults();

            for (FiducialResult tag : fiducials) {

                if (tag.getFiducialId() == 24) {

                    target = tag;

                    tx =
                            tag.getTargetXDegrees();

                    break;
                }
            }
        }


        // AIM

        turret.updateAuto(tx);


        // P/D TUNING

        // B = change step size
        if (gamepad1.bWasPressed()) {

            stepIndex =
                    (stepIndex + 1)
                            % stepSizes.length;
        }


        // D-pad left/right = P
        if (gamepad1.dpadLeftWasPressed()) {

            turret.setkP(
                    turret.getkP()
                            - stepSizes[stepIndex]
            );
        }

        if (gamepad1.dpadRightWasPressed()) {

            turret.setkP(
                    turret.getkP()
                            + stepSizes[stepIndex]
            );
        }


        // D-pad up/down = D
        if (gamepad1.dpadUpWasPressed()) {

            turret.setkD(
                    turret.getkD()
                            + stepSizes[stepIndex]
            );
        }

        if (gamepad1.dpadDownWasPressed()) {

            turret.setkD(
                    turret.getkD()
                            - stepSizes[stepIndex]
            );
        }


        // TELEMETRY

        if (target != null) {

            panelsTelemetry.addData(
                    "Tag",
                    target.getFiducialId()
            );

            panelsTelemetry.addData(
                    "TX",
                    tx
            );

            panelsTelemetry.addData(
                    "TY",
                    target.getTargetYDegrees()
            );

        } else {

            panelsTelemetry.addLine(
                    "Tag 24 not detected"
            );
        }

        panelsTelemetry.addLine(
                "---------------------------"
        );

        panelsTelemetry.addData(
                "P",
                turret.getkP()
        );

        panelsTelemetry.addData(
                "D",
                turret.getkD()
        );

        panelsTelemetry.addData(
                "Step Size",
                stepSizes[stepIndex]
        );

        panelsTelemetry.update();
    }


    @Override
    public void stop() {

        turret.stop();

        limelight.stop();
    }
}