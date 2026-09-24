package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.Foresight;
import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.Mecanum;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.pedropathing.revhub.localizers.PinpointLocalizer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("fleft");
        c.frontRightName.set("fright");
        c.backLeftName.set("bleft");
        c.backRightName.set("bright");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.manualBrakeMode.set(true);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-2.1688770684670278);
        c.yPodOffset.set(-8.599870336337354);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });

    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.2406500658503729);
                Controller secondaryTranslationalForward = Controller.proportional(0.08891376904352495);
                Controller primaryTranslationalLateral = Controller.proportional(0.3496807954660043);
                Controller secondaryTranslationalLateral = Controller.proportional(0.1291977102817494);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.016557228360837048));
                c.brake.set(Controller.proportionalFeedforward(0.01407364410671149));

                c.headingFeedback.set(Controller.proportional(4.319416509870432));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04574569968090677, 0.003952870041204273));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09384027470133786, 0.06807559784301168));
                c.quadraticBrakeCoefficients.set(Matrix.diag(4.251456416500241E-4, 9.250343903390457E-4));

                c.maxAchievableForwardVelocity.set(62.87890456358796);
                c.maxAchievableStrafeVelocity.set(51.44643242370265);
                c.naturalForwardDeceleration.set(33.02212152299688);
                c.naturalStrafeDeceleration.set(63.50151383464471);
            }
    );



    public static Follower create(HardwareMap h) {
        return new Follower(
                new PinpointLocalizer(h, localizerConfig),
                new Mecanum(h, drivetrainConfig),
                new Foresight(foresightConfig)
        );
    }
}