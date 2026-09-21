package org.firstinspires.ftc.teamcode.subSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

public class Shooter_Transfer {

    public DcMotor shooter;
    private Servo servo;

    public void init(HardwareMap hwMap) {
        shooter = hwMap.get(DcMotor.class, "shooter");
        shooter.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        shooter.setDirection(DcMotorSimple.Direction.REVERSE);
        servo = hwMap.get(Servo.class, "servo");
    }

    public static int servoState = 0;
    public static int shooterState = 0;

    public void setServoPosition(double position) {
        servo.setPosition(position);
    }


    public void loop() {
        Shooter_Transfer servo = new Shooter_Transfer();
        if (Shooter_Transfer.servoState == 1) {
            servo.setServoPosition(90);
        } else {
            servo.setServoPosition(0);
        }

        if (Shooter_Transfer.shooterState == 1) {
            shooter.setPower(1);
        }
        else {
            shooter.setPower(0);
        }
    }

}
