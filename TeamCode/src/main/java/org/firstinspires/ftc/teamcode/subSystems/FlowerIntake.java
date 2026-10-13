package org.firstinspires.ftc.teamcode.subSystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;

public class FlowerIntake {
    private final Servo arm;
    public FlowerIntake(HardwareMap hw) {
        arm = hw.get(Servo.class, "arm");
    }

    public Command lowerArm(){
        return instant(() -> arm.setPosition(0.0));
    }

    public Command raiseArm(){
        return instant(() -> arm.setPosition(90.0));
    }
}
