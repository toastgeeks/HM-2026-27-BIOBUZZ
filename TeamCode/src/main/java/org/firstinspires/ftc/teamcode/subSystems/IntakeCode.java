package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.function.DoubleSupplier;

import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

public class IntakeCode {
    private final DcMotorSimple motor;

    public IntakeCode(HardwareMap hw) {
        motor = hw.get(DcMotorSimple.class, "intake");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public Command run(double power) {
        return instant(() -> motor.setPower(power)).requiring(this);
    }

    public Command stop() {
        return run(0);
    }

    public Command manual(DoubleSupplier power) {
        return infinite(() -> motor.setPower(power.getAsDouble())).requiring(this);
    }
}