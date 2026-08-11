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
    /*
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
        if (index < orderedBalls.size - 1) {
            val nextBall = orderedBalls[index + 1]
            builder.setHeadingInterpolation(HeadingInterpolator.facingPoint(nextBall.x, nextBall.y))
        }

        val path = builder.build()
        follower.followPath(path, true)
    }

     */

}