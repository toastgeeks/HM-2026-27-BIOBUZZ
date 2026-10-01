package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

public class TurretMechanism {

    private CRServo turret;

    // ---------------- PD TUNING ----------------

    private double kP = 0.01;
    private double kD = 0.0;

    // We want TX to be 0 degrees
    private double goalX = 0.0;

    private double lastError = 0.0;

    // How close to center before stopping
    private double angleTolerance = 0.5;

    // Maximum CR servo power
    private double MAX_POWER = 0.6;

    private final ElapsedTime timer = new ElapsedTime();


    // ---------------- INITIALIZATION ----------------

    public void init(HardwareMap hwMap) {

        turret = hwMap.get(
                CRServo.class,
                "turret"
        );

        turret.setPower(0);
    }


    // ---------------- P TUNING ----------------

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }


    // ---------------- D TUNING ----------------

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }


    // ---------------- TIMER ----------------

    public void resetTimer() {
        timer.reset();
        lastError = 0;
    }


    // ---------------- AUTO AIM ----------------

    public void updateAuto(double tx) {

        // No target
        if (Double.isNaN(tx)) {
            stop();
            lastError = 0;
            return;
        }

        double deltaTime = timer.seconds();
        timer.reset();

        // Desired TX is 0
        double error = goalX - tx;

        // Proportional term
        double pTerm = error * kP;

        // Derivative term
        double dTerm = 0;

        if (deltaTime > 0) {
            dTerm =
                    ((error - lastError) / deltaTime)
                            * kD;
        }

        double power;

        // Close enough to target
        if (Math.abs(error) < angleTolerance) {

            power = 0;

        } else {

            power = Range.clip(
                    pTerm + dTerm,
                    -MAX_POWER,
                    MAX_POWER
            );
        }

        turret.setPower(power);

        lastError = error;
    }


    // ---------------- MANUAL MODE ----------------

    public void manual(double input) {

        turret.setPower(
                Range.clip(
                        input,
                        -MAX_POWER,
                        MAX_POWER
                )
        );
    }


    // ---------------- STOP ----------------

    public void stop() {
        turret.setPower(0);
    }
}