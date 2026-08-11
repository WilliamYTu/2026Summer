package org.firstinspires.ftc.teamcode
import com.bylazar.configurables.annotations.Configurable
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.Telemetry


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
}