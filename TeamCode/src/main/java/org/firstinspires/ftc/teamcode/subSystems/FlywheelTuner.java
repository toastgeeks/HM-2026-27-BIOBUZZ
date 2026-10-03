package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlywheelTuner extends OpMode{
    public DcMotorEx flyWheelMotor;

    public double highVelocity = 1500;
    public double lowVelocity = 900;

    double curTargetVelocity = highVelocity;

    double F = 0;
    double P = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};

    int stepIndex = 1;

    @Override
    public void init() {
        //flyWheelMotor1 = hardwareMap.get(DcMotorEx.class, "shooter1");
        //flyWheelMotor2 = hardwareMap.get(DcMotorEx.class, "shooter2");
        //flyWheelMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //flyWheelMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        //flyWheelMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flyWheelMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        //flyWheelMotor2.set
        telemetry.addLine("Init complete");
    }

    @Override
    public void loop() {
        // get all the gamepad commands
        // set target velocity
        // update telemetry

        if (gamepad2.dpadUpWasPressed()) {
            if (curTargetVelocity == highVelocity) {
                curTargetVelocity = lowVelocity;
            } else {curTargetVelocity = highVelocity; }
        }

        if (gamepad2.dpadDownWasPressed()) {
            stepIndex = (stepIndex + 1) % stepSizes.length;
        }

        if(gamepad2.yWasPressed()) {
            F += stepSizes[stepIndex];
        }
        if (gamepad2.aWasPressed()) {
            F -= stepSizes[stepIndex];
        }

        if (gamepad2.xWasPressed()) {
            P += stepSizes[stepIndex];
        }
        if (gamepad2.bWasPressed()) {
            P -= stepSizes[stepIndex];
        }

        //set new PIDF coefficients
        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);
        flyWheelMotor.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        //set velocity
        flyWheelMotor.setVelocity((curTargetVelocity));

        double curVelocity = flyWheelMotor.getVelocity();
        double error = curTargetVelocity - curVelocity;

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", "%.2f", curVelocity);
        telemetry.addData("Error", "%.2f", error);
        telemetry.addLine("--------------------------");
        telemetry.addData("Tuning P", "%.4f (Y/A)", P);
        telemetry.addData("Tuning F", "%.4f (X/B)", F);
        telemetry.addData("Step Size", "X.4f (Up Button)", stepSizes[stepIndex]);

    }
}
