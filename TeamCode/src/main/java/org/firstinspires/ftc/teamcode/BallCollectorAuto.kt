package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Drawing
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

@Autonomous(name = "Ball Collector Auto")
class BallCollectorAuto : LinearOpMode() {

    private val startPose = Pose(0.0, 0.0, 0.0)

    override fun runOpMode() {
        // hardwareMap and telemetry are inherited from LinearOpMode - no need to pass them in
        val limelight = Limelight(hardwareMap, telemetry)

        val follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)

        val drawing = Drawing()
        drawing.init()

        limelight.initLimelight()

        telemetry.addLine("Init complete - waiting for start")
        telemetry.update()

        limelight.start()

        waitForStart()
        if (isStopRequested) return

        if (limelight.displacementFromAngles().isEmpty()) {
            requestOpModeStop()
        } else {

            val fieldBalls: List<Pose> = limelight.displacementFromAngles().map { ball ->
                limelight.robotRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
            }
            val orderedBalls = limelight.shortestVisitOrder(follower.pose, fieldBalls)
            val pathChain = limelight.buildBallPathChain(follower, orderedBalls)
            follower.followPath(pathChain)

            while (opModeIsActive()) {
                drawing.drawDebug(follower)
                follower.update()
                telemetry.addData("X", follower.pose.x)
                telemetry.addData("Y", follower.pose.y)
                telemetry.addData("Heading", follower.pose.heading)
                telemetry.addData("point 1 x", orderedBalls[0].x)
                telemetry.addData("point 1 y", orderedBalls[0].y)
                telemetry.update()
            }
        }
        limelight.stop()
    }
}