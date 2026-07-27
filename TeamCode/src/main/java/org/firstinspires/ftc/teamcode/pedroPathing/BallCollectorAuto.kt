package org.firstinspires.ftc.teamcode

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.BezierLine
import com.pedropathing.geometry.Pose
import com.pedropathing.paths.PathChain
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import org.firstinspires.ftc.teamcode.pedroPathing.Constants
import kotlin.math.cos
import kotlin.math.hypot
import kotlin.math.sin

/**
 * Autonomous OpMode that:
 *  1. Uses the Limelight to snapshot every ball currently visible on the field.
 *  2. Converts each ball's robot-relative (forward, lateral) offset into an
 *     absolute field Pose.
 *  3. Brute-forces the shortest order in which to visit every ball.
 *  4. Builds a Pedro Pathing PathChain of Bezier curves through that order
 *     and drives it.
 *
 * NOTE: Detection happens once, right after waitForStart(). If balls can move
 * (e.g. pushed by other robots) after that snapshot, this strategy will drive
 * to stale positions - re-scan periodically if that's a concern for your game.
 */
@Autonomous(name = "Ball Collector Auto")
class BallCollectorAuto : LinearOpMode() {

    // TODO: set this to the robot's actual starting position/heading for your
    // alliance color and starting tile. (x inches, y inches, heading radians)
    private val startPose = Pose(72.0, 72.0, 0.0)

    override fun runOpMode() {
        val limelight = Limelight(hardwareMap, telemetry)
        val follower: Follower = Constants.createFollower(hardwareMap)

        limelight.initLimelight()
        follower.setStartingPose(startPose)

        telemetry.addLine("Init complete - waiting for start")
        telemetry.update()

        waitForStart()
        if (isStopRequested) return

        // --- 1. Detect balls relative to the robot ---
        limelight.start()
        val relativeBalls = limelight.displacementFromAngles()
        limelight.stop() // done with vision for this pass, free up the pipeline

        if (relativeBalls.isEmpty()) {
            telemetry.addLine("No balls detected - nothing to do")
            telemetry.update()
            return
        }

        // --- 2. Convert robot-relative offsets into absolute field Poses ---
        val fieldBalls: List<Pose> = relativeBalls.map { ball ->
            robotRelativeToFieldPose(follower.pose, ball.forward, ball.lateral)
        }

        // --- 3. Find the shortest order to visit every ball ---
        val orderedBalls = shortestVisitOrder(follower.pose, fieldBalls)

        // --- 4. Build and run the Bezier path chain through that order ---
        val pathChain = buildBallPathChain(follower, orderedBalls)
        follower.followPath(pathChain)

        while (opModeIsActive() && follower.isBusy()) {
            follower.update()
            telemetry.addData("Balls found", orderedBalls.size)
            telemetry.addData("Current X", follower.pose.x)
            telemetry.addData("Current Y", follower.pose.y)
            telemetry.update()
        }

        telemetry.addLine("Done - all balls visited")
        telemetry.update()
    }

    /**
     * Converts a ball offset that is relative to the robot's own forward/lateral
     * axes (as returned by [Limelight.displacementFromAngles]) into an absolute
     * field Pose, by rotating the offset by the robot's current field heading
     * and adding it to the robot's current field position.
     *
     * TODO: if the Limelight is mounted away from the robot's center of rotation,
     * add that physical offset here before rotating, for better accuracy.
     */
    private fun robotRelativeToFieldPose(robotPose: Pose, forward: Double, lateral: Double): Pose {
        val heading = robotPose.heading
        val fieldX = robotPose.x + forward * cos(heading) - lateral * sin(heading)
        val fieldY = robotPose.y + forward * sin(heading) + lateral * cos(heading)
        return Pose(fieldX, fieldY)
    }

    /**
     * Brute-forces every ordering of [points] (starting from [start]) and returns
     * the ordering with the smallest total straight-line travel distance.
     *
     * Fine for small ball counts (Limelight.displacementFromAngles caps out at 4,
     * so at most 24 permutations). If you ever raise that cap significantly,
     * swap this for a nearest-neighbor + 2-opt heuristic instead - brute force
     * is O(n!) and will not scale past ~8-9 points.
     */
    private fun shortestVisitOrder(start: Pose, points: List<Pose>): List<Pose> {
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

    private fun <T> permutations(items: List<T>): List<List<T>> {
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

    /**
     * Builds a Pedro Pathing PathChain made of Bezier curve segments (BezierLine,
     * the straight-line member of Pedro's Bezier curve family) starting at the
     * robot's current pose and passing through each ball position in order.
     * Heading is tangent to the direction of travel on each segment, so the
     * robot faces the next ball as it approaches - swap for
     * setLinearHeadingInterpolation(...) per segment if you need a fixed
     * intake-facing heading instead.
     */
    private fun buildBallPathChain(follower: Follower, orderedBalls: List<Pose>): PathChain {
        val builder = follower.pathBuilder()
        var previous = follower.pose

        for (ball in orderedBalls) {
            builder.addPath(BezierLine(previous, ball))
                .setTangentHeadingInterpolation()
            previous = ball
        }

        return builder.build()
    }
}