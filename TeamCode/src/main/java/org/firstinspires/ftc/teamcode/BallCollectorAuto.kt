package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import org.firstinspires.ftc.teamcode.Drawing
import kotlin.math.atan2
import kotlin.math.hypot

@Autonomous(name = "Ball Collector Auto")
class BallCollectorAuto : LinearOpMode() {


    private val startPose = Pose(72.0, 72.0, 0.0)
    private val filterSamples = 3

    private enum class Phase {DRIVING, TURNING}

    override fun runOpMode() {
        val follower = Constants.createFollower(hardwareMap)
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)
        follower.setStartingPose(startPose)

        val drawing = Drawing()
        drawing.init()
        limelight.initLimelight()
        limelight.start()

        waitForStart()
        if (isStopRequested) return

        if (limelight.displacementFromAngles().isEmpty()) {
            requestOpModeStop()
            return
        }
        follower.setMaxPower(0.75)

        val initialFieldBalls: List<Pose> = limelight.displacementFromAngles().map { ball ->
            follower.update()
            limelight.intakeRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
        }
        val orderedBalls: MutableList<Pose> = limelight.shortestVisitOrder(follower.pose, initialFieldBalls).toMutableList()

        var currentIndex = 0
        var targetHeading = 0.0
        var phase = Phase.DRIVING
        startDrive(follower, limelight, orderedBalls, currentIndex)
        while (opModeIsActive() && currentIndex < orderedBalls.size) {
            follower.update()
            drawing.drawDebug(follower)

            if (!follower.isBusy()){
                when (phase) {
                    Phase.DRIVING -> {
                        //TODO: Run intake sequence here
                        if (currentIndex < orderedBalls.size - 1){
                            val next = orderedBalls[currentIndex + 1]
                            val headingToNext = atan2(next.y - follower.pose.y, next.x - follower.pose.x)
                            targetHeading = headingToNext
                            telemetry.addData("Turning to", headingToNext)
                            follower.turnTo(headingToNext)
                            phase = Phase.TURNING
                        } else {
                            currentIndex++
                        }
                    }
                    Phase.TURNING -> {
                        val headingError = kotlin.math.abs(normalizeAngle(targetHeading - follower.pose.heading))
                        val settled = follower.angularVelocity < 0.01 && headingError < Math.toRadians(6.0)
                        if (settled) {
                            currentIndex++
                            phase = Phase.DRIVING
                            if (currentIndex < orderedBalls.size) {
                                startDrive(follower, limelight, orderedBalls, currentIndex)
                            }
                        }
                    }
                }
            }

            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Target Index: ", currentIndex)
            telemetry.addData("Phase", phase)
            telemetry.addData("isBusy", follower.isBusy())
            telemetry.addData("Heading (current)", follower.pose.heading)
            telemetry.update()
        }

        limelight.stop()
    }
    private fun startDrive(
        follower: Follower,
        limelight: Limelight,
        orderedBalls: MutableList<Pose>,
        index: Int
    ) {
        val targetPose = freshBallPose(limelight, follower, orderedBalls[index], filterSamples)
        orderedBalls[index] = targetPose
        val path = follower.pathBuilder()
            .addPath(BezierLine(follower::getPose, targetPose))
            .build()
        follower.followPath(path, true)
    }
    fun freshBallPose(limelight: Limelight, follower: Follower, fallback: Pose, filterSamples: Int): Pose {
        val readings = mutableListOf<Pose>()
        repeat(filterSamples) {
            val detections = limelight.displacementFromAngles()
            if (detections.isNotEmpty()) {
                val closest = detections.minByOrNull { hypot(it.forward, it.lateral) }!!
                readings.add(limelight.intakeRelativeToFieldPose(follower.pose, closest.forward, closest.lateral))
            }
        }
        if (readings.isEmpty()) return fallback
        return Pose(readings.map { it.x }.average(), readings.map { it.y }.average(), fallback.heading)
    }
    private fun normalizeAngle(angle: Double): Double {
        var a = angle
        while (a > Math.PI) a -= 2 * Math.PI
        while (a < -Math.PI) a += 2 * Math.PI
        return a
    }
}