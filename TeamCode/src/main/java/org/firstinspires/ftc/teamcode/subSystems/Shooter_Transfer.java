package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter_Transfer {

    public DcMotor shooter1;
    public DcMotor shooter2;
    private Servo servo;

    public void init(HardwareMap hwMap) {
        shooter1 = hwMap.get(DcMotor.class, "shooter1");
        shooter2 = hwMap.get(DcMotor.class, "shooter2");
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hwMap.get(Servo.class, "servo");
    }

    public static int servoState = 0;
    public static int shooterState = 0;

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }


    public void loop() {
        if (Shooter_Transfer.servoState == 1) {
            setServoPosition(0);
        } else {
            setServoPosition(0.25);
        }

        if (Shooter_Transfer.shooterState == 1) {
            shooter1.setPower(0.6);
            shooter2.setPower(0.6);
        }
        else {
            shooter1.setPower(0);
            shooter2.setPower(0);
        }

    }

}
