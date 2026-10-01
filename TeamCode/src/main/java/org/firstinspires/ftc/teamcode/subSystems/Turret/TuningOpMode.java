package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes.FiducialResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.List;

@TeleOp(name = "Turret - Tuning", group = "Turret")
public class TuningOpMode extends OpMode {

    private Limelight3A limelight;

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

        limelight = hardwareMap.get(
                Limelight3A.class,
                "limelight"
        );

        turret.init(hardwareMap);

        // AprilTag pipeline
        limelight.pipelineSwitch(0);

        telemetry.addLine(
                "Turret Tuning Mode"
        );

        telemetry.addLine(
                "Target: AprilTag 20"
        );

        telemetry.update();
    }


    @Override
    public void start() {

        limelight.start();

        turret.resetTimer();
    }


    @Override
    public void loop() {

        // -----------------------------------------
        // GET LIMELIGHT RESULT
        // -----------------------------------------

        LLResult result =
                limelight.getLatestResult();

        double tx = Double.NaN;

        FiducialResult target = null;


        // -----------------------------------------
        // FIND TAG 20
        // -----------------------------------------

        if (result != null && result.isValid()) {

            List<FiducialResult> fiducials =
                    result.getFiducialResults();

            for (FiducialResult tag : fiducials) {

                if (tag.getFiducialId() == 20) {

                    target = tag;

                    tx =
                            tag.getTargetXDegrees();

                    break;
                }
            }
        }


        // -----------------------------------------
        // AIM
        // -----------------------------------------

        turret.updateAuto(tx);


        // -----------------------------------------
        // P/D TUNING
        // -----------------------------------------

        // B = change step size
        if (gamepad2.bWasPressed()) {

            stepIndex =
                    (stepIndex + 1)
                            % stepSizes.length;
        }


        // D-pad left/right = P
        if (gamepad2.dpadLeftWasPressed()) {

            turret.setkP(
                    turret.getkP()
                            - stepSizes[stepIndex]
            );
        }

        if (gamepad2.dpadRightWasPressed()) {

            turret.setkP(
                    turret.getkP()
                            + stepSizes[stepIndex]
            );
        }


        // D-pad up/down = D
        if (gamepad2.dpadUpWasPressed()) {

            turret.setkD(
                    turret.getkD()
                            + stepSizes[stepIndex]
            );
        }

        if (gamepad2.dpadDownWasPressed()) {

            turret.setkD(
                    turret.getkD()
                            - stepSizes[stepIndex]
            );
        }


        // -----------------------------------------
        // TELEMETRY
        // -----------------------------------------

        if (target != null) {

            telemetry.addData(
                    "Tag",
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

        } else {

            telemetry.addLine(
                    "Tag 20 not detected"
            );
        }

        telemetry.addLine(
                "---------------------------"
        );

        telemetry.addData(
                "P",
                "%.5f",
                turret.getkP()
        );

        telemetry.addData(
                "D",
                "%.5f",
                turret.getkD()
        );

        telemetry.addData(
                "Step Size",
                "%.5f",
                stepSizes[stepIndex]
        );

        telemetry.update();
    }


    @Override
    public void stop() {

        turret.stop();

        limelight.stop();
    }
}