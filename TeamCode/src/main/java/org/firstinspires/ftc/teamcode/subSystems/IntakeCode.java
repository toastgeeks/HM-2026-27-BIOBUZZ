package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeCode {

    private DcMotor intake;

    public void init(HardwareMap hwMap) {
        intake = hwMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void setIntakeSpeed(double power) {
        intake.setPower(power);
    }

    public void stop() {
        intake.setPower(0);
    }
}