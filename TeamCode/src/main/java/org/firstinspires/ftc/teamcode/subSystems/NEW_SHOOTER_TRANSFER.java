package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.sequential;

public class NEW_SHOOTER_TRANSFER {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx transfer;
    private Servo gate;
    public double shooterSpeed = 0.65;
    private double servoPose = 0.5;

    public NEW_SHOOTER_TRANSFER(HardwareMap hw) {
        shooter1 = hw.get(DcMotorEx.class, "shooter1");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2 = hw.get(DcMotorEx.class, "shooter2");
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        transfer = hw.get(DcMotorEx.class, "transfer");
        transfer.setDirection(DcMotorSimple.Direction.REVERSE);
        gate = hw.get(Servo.class, "gate");

    }

    public void newShooterSpeed(double newShooterSpeed) {
        this.shooterSpeed += newShooterSpeed;
    }

    public Command rev() {
        return instant(() -> {
            shooter1.setPower(shooterSpeed);
            shooter2.setPower(shooterSpeed);
        }).requiring(this);
    }

    public Command stopRev() {
        return instant(() -> {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }).requiring(this);
    }

    public Command stop() {
        return instant(() -> {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }).requiring(this);
    }

    public Command fire(double transferTime) {
        return sequential(
                instant(() -> {
                    if (shooter1.getPower() == 0) {
                        shooter1.setPower(shooterSpeed);
                        shooter2.setPower(shooterSpeed);
                    }
                }),
                instant(() -> gate.setPosition(0.3)),
                instant(() -> transfer.setPower(1.0)),
                waitMs(transferTime),
                instant(() -> transfer.setPower(0.0)),
                instant(() -> shooter1.setPower(0.0)),
                instant(() -> shooter2.setPower(0.0)),
                instant(() -> gate.setPosition(0.5))
        ).requiring(this);
    }

    public Command transfer(double transferSpeed) {
        return instant(() -> transfer.setPower(transferSpeed)).requiring(this);
    }

    public Command transferStop() {
        return instant(() -> {
            transfer.setPower(0);
        }).requiring(this);
    }

    public void newServoPose(double newServoPose) {
        this.servoPose = newServoPose;
        this.gate.setPosition(this.servoPose);
    }
    public double getServoPose() { return this.servoPose; }
    public Command servoTuning() {
        return instant(() -> {
            gate.setPosition(servoPose);
        }).requiring(this);
    }

}
