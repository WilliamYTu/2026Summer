package org.firstinspires.ftc.teamcode
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.HeadingInterpolator
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.hypot


@Configurable
class Robot(private val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    val limelight = Limelight(hardwareMap, telemetry)
    // default pipeline is 1 not 0 for this config

    fun limelightToggle(){
        if (limelight.state == Limelight.LimelightState.OFF){
            limelight.start()
        } else{
            limelight.stop()
        }
    }

    fun changeLimelightPos(change: Double){
        val previousPos = limelight.getServoPosition()
        limelight.limelightServoPos(previousPos + change)
    }

    fun buildLineTo(follower: Follower, target: Pose): PathChain {
        return follower.pathBuilder()
            .addPath(BezierLine(follower.pose, target))
            .setLinearHeadingInterpolation(follower.pose.heading, target.heading)
            .build()
    }

    fun startSegment(
        follower: Follower,
        limelight: Limelight,
        orderedBalls: List<Pose>,
        index: Int,
        filterSamples: Int
    ) {
        val targetPose = freshBallPose(limelight, follower, orderedBalls[index], filterSamples)
        val builder = follower.pathBuilder()
            .addPath(
                BezierLine(
                    follower::getPose,
                    targetPose
                )
            )

        // Only force a "face the next ball" heading if there IS a next ball.
        // On the last ball, don't override heading at all - let the robot end up
        // facing whatever direction it naturally settles at.
        if (index < orderedBalls.size - 1) {
            val nextBall = orderedBalls[index + 1]
            builder.setHeadingInterpolation(HeadingInterpolator.facingPoint(nextBall.x, nextBall.y))
        }

        val path = builder.build()
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
}