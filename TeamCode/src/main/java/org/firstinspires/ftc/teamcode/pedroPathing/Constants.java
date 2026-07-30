package org.firstinspires.ftc.teamcode.pedroPathing;


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
// import org.firstinspires.ftc.teamcode.subsystems.Robot;


public class Constants {
    TwoWheelConstants myConstants = new TwoWheelConstants();



    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.61)


            .lateralZeroPowerAcceleration(-8)
            .forwardZeroPowerAcceleration(-7)

            .headingPIDFCoefficients(new PIDFCoefficients(1, 0.0, 0.05, 0.05))
            .useSecondaryHeadingPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.8, 0.0, 0.1, 0.02))
            .headingPIDFSwitch(0.7670796327)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.015, 0.05))
            .useSecondaryTranslationalPIDF(true)
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.02, 0.02))

            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0.0, 0.0002, 0.6, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.04, 0.0, 0.00001, 0.6, 0.02))
            .useSecondaryDrivePIDF(true)
            .drivePIDFSwitch(20)


            ;



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("fr")
            .rightRearMotorName("br")
            .leftRearMotorName("bl")
            .leftFrontMotorName("fl")
            .xVelocity(78.00285315556257)
            .yVelocity(62.8918609819)
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
            .forwardPodY(7.0)
            .strafePodX(0.0)
            .forwardTicksToInches(0.0014512057)
            .strafeTicksToInches(0.007273707)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
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