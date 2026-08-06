package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.pedroPathing.Constants

@TeleOp(name = "Ball Detector")
class Test : LinearOpMode() {

    override fun runOpMode() {
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)
        limelight.initLimelight()
        limelight.start()
        val follower: Follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(Pose(0.0, 0.0, 0.0))
        waitForStart()


        while (opModeIsActive()) {
            limelight.getBallNumber()
            if (gamepad1.a){
                robot.changeLimelightPos(-0.1)
            }
            if (gamepad1.y){
                robot.changeLimelightPos(0.1)
            }
            if (gamepad1.x){
                robot.limelightToggle()
            }

            val result = limelight.displacementFromAngles()


            telemetry.addData("Servo position", limelight.getServoPosition())

            val rawResult = limelight.getlatestResult()
            telemetry.addData("Valid", rawResult?.isValid)
            telemetry.update()

            val firstBall = result.getOrNull(0)
            val secondBall = result.getOrNull(1)
            val thirdBall = result.getOrNull(2)
            val fourthBall = result.getOrNull(3)


            if (firstBall != null) {
                telemetry.addData("Forward1: ", firstBall.forward)
                telemetry.addData("Lateral1: ", firstBall.lateral)
                telemetry.addData("Distance1: ", firstBall.distance)
                telemetry.addData("Coordinates1: ", limelight.robotRelativeToFieldPose(follower.pose, firstBall.forward, firstBall.lateral))
            }
            if (secondBall != null) {
                telemetry.addData("Forward2: ", secondBall.forward)
                telemetry.addData("Lateral2: ", secondBall.lateral)
                telemetry.addData("Distance2: ", secondBall.distance)
            }
            if (thirdBall != null) {
                telemetry.addData("Forward3: ", thirdBall.forward)
                telemetry.addData("Lateral3: ", thirdBall.lateral)
                telemetry.addData("Distance3: ", thirdBall.distance)
            }
            if (fourthBall != null) {
                telemetry.addData("Forward4: ", fourthBall.forward)
                telemetry.addData("Lateral4: ", fourthBall.lateral)
                telemetry.addData("Distance4: ", fourthBall.distance)
            }
            telemetry.update()
        }
    }
}