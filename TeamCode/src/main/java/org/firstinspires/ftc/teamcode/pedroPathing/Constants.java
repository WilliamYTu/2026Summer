package org.firstinspires.ftc.teamcode.pedroPathing;


import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import kotlin.jvm.JvmField;
// import org.firstinspires.ftc.teamcode.subsystems.Robot;

public class Constants {
    TwoWheelConstants myConstants = new TwoWheelConstants();

    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-39.68251553190508)
            .lateralZeroPowerAcceleration(-50.35198018424342)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1, 0.0, 0.01, 0.025))
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3, 0.0, 0.005, 0.04))
            .useSecondaryTranslationalPIDF(true)
            .translationalPIDFSwitch(6)


            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.0, 0.0, 0.1, 0.01))
            .headingPIDFCoefficients(new PIDFCoefficients(5, 0.0, 0.1, 0.02))
            .useSecondaryHeadingPIDF(true)
            .headingPIDFSwitch(0.5580796327)

            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.004, 0.0, 0.00035      , 0.6, 0.03))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.05, 0.0, 0.00001, 0.6, 0.02))
            .useSecondaryDrivePIDF(true)
            .drivePIDFSwitch(30)

            .mass(11.3398);






    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.5)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .xVelocity(73.38503752726977)
            .yVelocity(56.583242400627924)
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE) //good
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE) //good
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);

    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardEncoder_HardwareMapName("bl")
            .strafeEncoder_HardwareMapName("fr")
            .forwardEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .forwardPodY(-6.75)
            .strafePodX(0.0)
            .forwardTicksToInches(5.220301935654224E-4)
            .strafeTicksToInches(0.002336108777590704)
            .IMU_HardwareMapName("revimu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                            RevHubOrientationOnRobot.UsbFacingDirection.UP
                    )


            );

    public static PathConstraints pathConstraints = new PathConstraints(
            0.99,

            100,
            2,
            1.25);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .twoWheelLocalizer(localizerConstants)
                .build();
    }


}