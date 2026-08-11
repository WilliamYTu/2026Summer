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
    private val filterSamples = 5

    override fun runOpMode() {
        val limelight = Limelight(hardwareMap, telemetry)
        val robot = Robot(hardwareMap, telemetry)
        val follower = Constants.createFollower(hardwareMap)
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
        follower.setMaxPower(0.5)

        val initialFieldBalls: List<Pose> = limelight.displacementFromAngles().map { ball ->
            follower.update()
            limelight.intakeRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
        }
        val orderedBalls = limelight.shortestVisitOrder(follower.pose, initialFieldBalls)

        var currentIndex = 0
        robot.startSegment(follower, limelight, orderedBalls, currentIndex, filterSamples)



        while (opModeIsActive() && currentIndex < orderedBalls.size) {
            follower.update()
            drawing.drawDebug(follower)

            if (!follower.isBusy()){
                //TODO: INTAKE SEQUENCE GOES HERE
                currentIndex++
                if (currentIndex < orderedBalls.size) {
                    robot.startSegment(follower, limelight, orderedBalls, currentIndex, filterSamples)
                }
            }


            telemetry.addData("X", follower.pose.x)
            telemetry.addData("Y", follower.pose.y)
            telemetry.addData("Heading", follower.pose.heading)
            telemetry.addData("Target Index: ", currentIndex)
            telemetry.update()
        }

        limelight.stop()
    }
}