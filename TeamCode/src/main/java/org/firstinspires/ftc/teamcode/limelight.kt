package org.firstinspires.ftc.teamcode

import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.geometry.BezierPoint
import com.pedropathing.paths.PathChain
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.Math.toRadians
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

import kotlin.math.tan
import kotlin.math.sqrt
// import kotlin.math.toRadians

@Configurable
class Limelight(private val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    private lateinit var limelight: Limelight3A
    private lateinit var imu: IMU

    private  val cameraHeightInches = 11.0
    private  val ballHeightInches = 1.5     // measure from center of the ball to ground
    private  val safety = -999.0
    // private val k = 75.78304131
    var state = LimelightState.OFF

    val limelightServo = (hardwareMap["limelightServo"] as ServoImplEx).apply {
        direction = Servo.Direction.FORWARD
        // pwmRange = PwmControl.PwmRange(520.0, 2480.0)
    }

    fun initLimelight() {
        limelight = hardwareMap.get(Limelight3A::class.java, "Ethernet Device")
        limelight.pipelineSwitch(2)

        limelightServoPos(servoPos)
    }

    fun limelightServoPos(pos: Double){
        limelightServo.position = pos
    }


    fun getServoPosition(): Double {
        return limelightServo.position
    }

    fun start(){
        state = LimelightState.ON
        limelight.start()
    }

    fun stop(){
        state = LimelightState.OFF
        limelight.stop()
    }

    fun getBallNumber(): Int{
        val result = displacementFromAngles()
        return if (result == emptyList<BallPosition>()) {
            -1
        } else {
            result.size
        }
    }

    data class BallPosition(val forward: Double, val lateral: Double, val distance: Double)

    fun displacementFromAngles(): List<BallPosition> {
        val result: LLResult? = limelight.latestResult
        // limelight.updatePythonInputs()
        if (result != null) {
            val pythonOutput = result.pythonOutput ?: return emptyList()

            val points = pythonOutput.take(8)
                .chunked(2)
                .takeWhile { pair -> pair.size == 2 && safety !in pair }

            val ballPositions: List<BallPosition> = points.map { pair ->
                val tx = pair[0]
                val ty = pair[1]

                val angleToTargetRadians = toRadians(cameraAngleDegrees + ty)
                val forward = (cameraHeightInches - ballHeightInches) * tan(angleToTargetRadians)
                val lateral = -(forward * tan(toRadians(tx)))
                val distance = sqrt(forward * forward + lateral * lateral)// distance formula
                BallPosition(forward, lateral, distance)
            }
            return ballPositions
        }
        return emptyList()
    }

    fun getlatestResult(): LLResult? {
        return limelight.latestResult
    }

    fun returnTelemetry(){
        val result = displacementFromAngles()

        val firstBall = result.getOrNull(0)
        val secondBall = result.getOrNull(0)
        val thirdBall = result.getOrNull(0)
        val fourthBall = result.getOrNull(0)


        if (firstBall != null) {
            telemetry.addData("Forward: ", firstBall.forward)
            telemetry.addData("Lateral: ", firstBall.lateral)
            telemetry.addData("Distance: ", firstBall.distance)
        }
        if (secondBall != null) {
            telemetry.addData("Forward: ", secondBall.forward)
            telemetry.addData("Lateral: ", secondBall.lateral)
            telemetry.addData("Distance: ", secondBall.distance)
        }
        if (thirdBall != null) {
            telemetry.addData("Forward: ", thirdBall.forward)
            telemetry.addData("Lateral: ", thirdBall.lateral)
            telemetry.addData("Distance: ", thirdBall.distance)
        }
        if (fourthBall != null) {
            telemetry.addData("Forward: ", fourthBall.forward)
            telemetry.addData("Lateral: ", fourthBall.lateral)
            telemetry.addData("Distance: ", fourthBall.distance)
        }
    }

    fun robotRelativeToFieldPose(robotPose: Pose, forward: Double, lateral: Double): Pose {
        val heading = robotPose.heading
        val fieldX = robotPose.x + forward * cos(heading) - lateral * sin(heading)
        val fieldY = robotPose.y + forward * sin(heading) + lateral * cos(heading)
        return Pose(fieldX, fieldY)
    }

    fun intakeRelativeToFieldPose(robotPose: Pose, forward: Double, lateral: Double, pullBackDistance: Double = 0.0): Pose {
        val heading = robotPose.heading
        val targetX = robotPose.x + forward * cos(heading) - lateral * sin(heading)
        val targetY = robotPose.y + forward * sin(heading) + lateral * cos(heading)

        val dx = targetX - robotPose.x
        val dy = targetY - robotPose.y
        val dist = hypot(dx, dy)

        // Scale the vector so its length is reduced by pullBackDistance, clamped at 0
        val scale = if (dist > 1e-6) (dist - pullBackDistance).coerceAtLeast(0.0) / dist else 0.0

        val fieldX = robotPose.x + dx * scale
        val fieldY = robotPose.y + dy * scale
        return Pose(fieldX, fieldY)
    }

    fun shortestVisitOrder(start: Pose, points: List<Pose>): List<Pose> {
        if (points.size <= 1) return points

        var bestOrder = points
        var bestDistance = Double.MAX_VALUE

        for (order in permutations(points)) {
            var distance = 0.0
            var current = start
            for (point in order) {
                distance += hypot(point.x - current.x, point.y - current.y)
                current = point
            }
            if (distance < bestDistance) {
                bestDistance = distance
                bestOrder = order
            }
        }
        return bestOrder
    }

    fun <T> permutations(items: List<T>): List<List<T>> {
        if (items.size <= 1) return listOf(items)
        val result = mutableListOf<List<T>>()
        for (i in items.indices) {
            val remaining = items.toMutableList().apply { removeAt(i) }
            for (perm in permutations(remaining)) {
                result.add(listOf(items[i]) + perm)
            }
        }
        return result
    }

    fun buildBallPathChain(follower: Follower, orderedBalls: List<Pose>): PathChain {
        val builder = follower.pathBuilder()
        var previous = follower.pose

        for (ball in orderedBalls) {
            builder.addPath(BezierLine(previous, ball))
                .setTangentHeadingInterpolation()
            previous = ball
        }

        return builder.build()
    }

    fun averagedBallPose(limelight: Limelight, robotPose: Pose, samples: Int): Pose? {
        val readings = mutableListOf<Pose>()
        repeat(samples) {
            val detections = limelight.displacementFromAngles()
            if (detections.isNotEmpty()) {
                val closest = detections.minByOrNull { hypot(it.forward, it.lateral) }!!
                readings.add(limelight.intakeRelativeToFieldPose(robotPose, closest.forward, closest.lateral))
            }
        }
        if (readings.isEmpty()) return null
        val avgX = readings.map { it.x }.average()
        val avgY = readings.map { it.y }.average()
        return Pose(avgX, avgY, readings.last().heading)
    }


    enum class LimelightState {
        ON, OFF
    }


    companion object {
        @JvmField
        var cameraAngleDegrees = 72.0

        @JvmField
        var servoPos = 0.25
    }


}