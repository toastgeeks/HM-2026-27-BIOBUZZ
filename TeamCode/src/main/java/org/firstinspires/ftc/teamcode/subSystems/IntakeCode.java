package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;

public class IntakeCode {
    private final DcMotor intake;

    public IntakeCode(HardwareMap hw) {
        intake = hw.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
    }

    public Command run(double power) {
        return instant(() -> intake.setPower(power)).requiring(this);
    }

    public Command stop() {
        return run(0);
    }
}
