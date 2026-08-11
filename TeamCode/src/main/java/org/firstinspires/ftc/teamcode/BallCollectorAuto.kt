package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.util.ElapsedTime
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Drawing
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin
@Autonomous(name = "Ball Collector Auto")
class BallCollectorAuto : LinearOpMode() {

    private val startPose = Pose(72.0, 72.0, 0.0)

    override fun runOpMode() {
        // hardwareMap and telemetry are inherited from LinearOpMode - no need to pass them in
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)

        val follower = Constants.createFollower(hardwareMap)
        follower.setStartingPose(startPose)

        val drawing = Drawing()
        drawing.init()

        val retargetIntervalMs = 500L      // how often we're allowed to re-issue a path
        val retargetThreshold = 1.5        // inches of drift needed before we bother re-pathing
        val lockDistance = 8.0             // once this close to target, stop re-targeting, in inches
        val filterSamples = 5

        limelight.initLimelight()

        telemetry.addLine("Init complete - waiting for start")
        telemetry.update()

        limelight.start()

        waitForStart()
        if (isStopRequested) return

        if (limelight.displacementFromAngles().isEmpty()) {
            requestOpModeStop()
            return
        }
        follower.setMaxPower(0.5)


        val initialFieldBalls: List<Pose> = limelight.displacementFromAngles().map { ball ->
            follower.update()
            limelight.intakeRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
        }
        val orderedBalls = limelight.shortestVisitOrder(follower.pose, initialFieldBalls).toMutableList()

        var currentIndex = 0
        var currentTarget = orderedBalls[0]
        val retargetTimer = ElapsedTime()

        follower.followPath(robot.buildLineTo(follower, currentTarget), true)


        while (opModeIsActive() && currentIndex < orderedBalls.size) {
            drawing.drawDebug(follower)
            follower.update()

            val distToTarget = hypot(
                currentTarget.x - follower.pose.x,
                currentTarget.y - follower.pose.y
            )

            if (distToTarget > lockDistance && retargetTimer.milliseconds() > retargetIntervalMs) {
                val freshEstimate = limelight.averagedBallPose(limelight, follower.pose, filterSamples)
                if (freshEstimate != null) {
                    val drift = hypot(
                        freshEstimate.x - currentTarget.x,
                        freshEstimate.y - currentTarget.y
                    )
                    if (drift > retargetThreshold) {
                        currentTarget = freshEstimate
                        orderedBalls[currentIndex] = freshEstimate
                        follower.followPath(robot.buildLineTo(follower, currentTarget), true)
                    }
                }
                retargetTimer.reset()
            }

            if (!follower.isBusy()){
                //TODO: INTAKE SEQUENCE GOES HERE
                currentIndex++
                if (currentIndex < orderedBalls.size) {
                    currentTarget = orderedBalls[currentIndex]
                    follower.followPath(robot.buildLineTo(follower, currentTarget), true)
                }
            }


            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Target Index: ", currentIndex)
            telemetry.addData("Target Distance: ", distToTarget)
            telemetry.update()
        }

        limelight.stop()
    }
}