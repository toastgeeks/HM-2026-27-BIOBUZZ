package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;

public class FlowerIntake {
    private final Servo arm;

    private static final double LOWER_POSITION = 0.37;
    private static final double RAISE_POSITION = 0.7;

    public FlowerIntake(HardwareMap hw) {
        arm = hw.get(Servo.class, "arm");
    }

    public Command lowerArm() {
        return instant(() -> arm.setPosition(LOWER_POSITION)).requiring(this);
    }

    public Command raiseArm() {
        return instant(() -> arm.setPosition(RAISE_POSITION)).requiring(this);
    }
}
