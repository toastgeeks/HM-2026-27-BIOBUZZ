package org.firstinspires.ftc.teamcode.subSystems.Turret;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

public class TurretMechanism {
    private CRServo turret;

    private double kP = 0.0001;
    private double kD = 0.0000;
    private double goalX = 0;
    private double lastError = 0;
    private double angleTolerance = 0.2;
    private double MAX_POWER = 0.6;
    private double power = 0;

    private final ElapsedTime timer = new ElapsedTime();

    public void init(HardwareMap hwMap) {
        turret = hwMap.get(CRServo.class, "turret");
        turret.setPower(0);
    }

    public void setkP(double newKP) {
        kP = newKP;
    }

    public double getkP() {
        return kP;
    }

    public void setkD(double newKD) {
        kD = newKD;
    }

    public double getkD() {
        return kD;
    }

    public void resetTimer(){
        timer.reset();
        lastError = 0;
    }

    public void update(double tx) {
        double deltaTime = timer.seconds();
        timer.reset();

        //no tag detected
        if (Double.isNaN(tx)) {
            turret.setPower(0);
            lastError = 0;
            return;
        }

        // ------ Start PD controller -------

        double error = goalX - tx;
        double pTerm = error * kP;

        double dTerm = 0;
        if (deltaTime > 0) {
            dTerm = ((error - lastError) /deltaTime) * kD;
        }

        if (Math.abs(error) < angleTolerance) {
            power = 0;
        } else {
            power = Range.clip(pTerm + dTerm, -MAX_POWER, MAX_POWER);
        }

        // safety encoder check OR magnetic limit switch check, OR other safeties

        turret.setPower(power);
        lastError = error;
    }

}
