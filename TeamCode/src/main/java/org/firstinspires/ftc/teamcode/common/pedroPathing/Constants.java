package org.firstinspires.ftc.teamcode.common.pedroPathing;
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
        c.frontLeftName.set("frontLeft");
        c.frontRightName.set("frontRight");
        c.backLeftName.set("backLeft");
        c.backRightName.set("backRight");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backRightDirection.set(DcMotorSimple.Direction.FORWARD);
    });

    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-0.8444454914002907);
        c.yPodOffset.set(-1.4199307959849443);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.REVERSED);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29850419692168856);
                Controller secondaryTranslationalForward = Controller.proportional(0.11028932458352277);
                Controller primaryTranslationalLateral = Controller.proportional(0.4202392446560423);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15526717190098296);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.01742051048284047));
                c.brake.set(Controller.proportionalFeedforward(0.014807433910414398));

                c.headingFeedback.set(Controller.proportional(4.15893142438078));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0696102069465161, 0.0019204469381248778));

                c.linearBrakeCoefficients.set(Matrix.diag(0.09680437655781574, 0.015590178005521352));
                c.quadraticBrakeCoefficients.set(Matrix.diag(0.0012873170381057317, -4.398277923828323E-5));

                c.maxAchievableForwardVelocity.set(63.21891090763959);
                c.maxAchievableStrafeVelocity.set(48.58468897330256);
                c.naturalForwardDeceleration.set(36.72337530073975);
                c.naturalStrafeDeceleration.set(65.67411276782468);
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