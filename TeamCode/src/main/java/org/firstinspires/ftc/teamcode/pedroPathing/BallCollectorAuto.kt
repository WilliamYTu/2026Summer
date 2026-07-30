package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Limelight

@Autonomous(name = "Ball Collector Auto")
class BallCollectorAuto : LinearOpMode() {

    private val startPose = Pose(0.0, 0.0, 0.0)

    override fun runOpMode() {
        // hardwareMap and telemetry are inherited from LinearOpMode - no need to pass them in
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)
        val follower: Follower = Constants.createFollower(hardwareMap)

        follower.setStartingPose(startPose)
        limelight.initLimelight()

        telemetry.addLine("Init complete - waiting for start")
        telemetry.update()

        limelight.start()

        waitForStart()
        if (isStopRequested) return

        if (limelight.displacementFromAngles().isEmpty()) {
            telemetry.addLine("No balls detected - nothing to do")
            telemetry.update()
        } else {
            telemetry.addLine("Balls detected - converting to field coordinates")
            telemetry.update()

            val fieldBalls: List<Pose> = limelight.displacementFromAngles().map { ball ->
                limelight.robotRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
            }

            val orderedBalls = limelight.shortestVisitOrder(follower.pose, fieldBalls)
            telemetry.addLine("Balls ordered - building path chain")
            telemetry.update()

            val pathChain = limelight.buildBallPathChain(follower, orderedBalls)
            follower.followPath(pathChain)

            // This is the piece that was missing: keep pumping update()
            // every cycle until the follower reports it's done with the path.
            while (opModeIsActive()) {
                follower.update()
                telemetry.addData("X", follower.pose.x)
                telemetry.addData("Y", follower.pose.y)
                telemetry.addData("Heading", follower.pose.heading)
                telemetry.update()
            }
        }

        telemetry.addLine("Done - all balls visited")
        telemetry.update()
        limelight.stop()
    }
}