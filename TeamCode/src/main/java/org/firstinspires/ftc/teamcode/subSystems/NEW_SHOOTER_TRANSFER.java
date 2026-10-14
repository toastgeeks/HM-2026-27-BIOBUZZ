package org.firstinspires.ftc.teamcode.subSystems;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class NEW_SHOOTER_TRANSFER {
    private DcMotorEx shooter;
    private DcMotorEx transfer;
    //private Servo gate;
    public double shooterSpeed;

    public NEW_SHOOTER_TRANSFER(HardwareMap hw) {
        shooter = hw.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        transfer = hw.get(DcMotorEx.class, "transfer");
    }

    public Command rev() {
        return instant(() -> shooter.setPower(shooterSpeed)).requiring(this);
    }

    public Command stop() {
        return instant(() -> {
            shooter.setPower(0);
        }).requiring(this);
    }

    public Command fire() {
        return sequential(
                instant(() -> {
                    if (shooter.getPower() == 0) {
                        shooter.setPower(shooterSpeed);
                    }
                }),
                instant(() -> transfer.setPower(1.0)),
                waitMs(500),
                instant(() -> transfer.setPower(0.0)),
                instant(() -> shooter.setPower(0.0))
        ).requiring(this);
    }

}
