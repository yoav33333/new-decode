package org.firstinspires.ftc.teamcode.Pedro;

import static org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.measurementVariance;
import static org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars.processVariance;
//import static org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.nominaVoltage;
import static org.firstinspires.ftc.teamcode.Subsystems.Robot.RobotVars.nominalVoltage;
import static org.firstinspires.ftc.teamcode.Util.Util.mmToInches;
import static java.lang.Math.PI;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.ftc.localization.localizers.TwoWheelLocalizer;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Subsystems.DriveSubsystem.DriveVars;

import dev.nextftc.ftc.ActiveOpMode;
import kotlin.Lazy;

public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(10)
            .forwardZeroPowerAcceleration(-36.9823349580575)
            .lateralZeroPowerAcceleration(-74.28974372367291)
            .useSecondaryTranslationalPIDF(false)
            .useSecondaryHeadingPIDF(false)
            .useSecondaryDrivePIDF(false)
            .centripetalScaling(0.0004)
            .automaticHoldEnd(true)
            .turnHeadingErrorThreshold(0.01)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.001, 0.045))
            .headingPIDFCoefficients(new PIDFCoefficients(0.85, 0, 0.08, 0.06))
            .drivePIDFCoefficients(
                    new FilteredPIDFCoefficients(0.015, 0, 0.0035, 0.95, 0.05)
            );

    public static MecanumConstants driveConstants = new MecanumConstants()
            .leftFrontMotorName("lf")
            .leftRearMotorName("lb")
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(69.04463900656683)
            .yVelocity(52.52714724615654)
            .useBrakeModeInTeleOp(true)
            .useVoltageCompensation(true)
            .nominalVoltage(nominalVoltage);

    public static TwoWheelConstants deadWheelLocalizerConstants =
            new TwoWheelConstants()
                    .IMU_HardwareMapName("imu")
                    .IMU_Orientation(
                            new RevHubOrientationOnRobot(
                                    RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
                                    RevHubOrientationOnRobot.UsbFacingDirection.DOWN
                            )
                    )
                    .forwardTicksToInches((mmToInches(35) * PI) / 8192)
                    .strafeTicksToInches((mmToInches(35) * PI) / 8192)
                    .forwardPodY(-mmToInches(210.0-43))
                    .strafePodX(mmToInches(182.3-33))
                    .strafeEncoder_HardwareMapName("lf")
                    .forwardEncoder_HardwareMapName("lb")
                    .strafeEncoderDirection(Encoder.FORWARD)
                    .forwardEncoderDirection(Encoder.REVERSE);
//    public static FusionLocalizer createFusionLocalize(HardwareMap hardwareMap) {
//        return new FusionLocalizer(
//                new TwoWheelLocalizer(hardwareMap, deadWheelLocalizerConstants),
//                new double[]{0.05, 0.05, Math.toRadians(0.05)},
//                new double[]{1, 1, Math.toRadians(5)},
//                5);
//    }
    public static Lazy<FusionLocalizer> localizer = kotlin.LazyKt.lazy(
            ()-> new FusionLocalizer(
                new TwoWheelLocalizer(ActiveOpMode.hardwareMap(), deadWheelLocalizerConstants),
                DriveVars.P,
            processVariance,
                    measurementVariance,
                    500
            ));
    public static AcclTwoWheelLocalizer getLocalizer(HardwareMap hmap, TwoWheelConstants constants){
        return new AcclTwoWheelLocalizer(hmap, constants);
    }

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,
            10,
            1.75,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
//                .setLocalizer(localizer.getValue())
//                .setLocalizer(getLocalizer(hardwareMap, deadWheelLocalizerConstants))
                .twoWheelLocalizer(deadWheelLocalizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}
