package org.firstinspires.ftc.teamcode

import com.bylazar.field.PanelsField.field
import com.bylazar.field.PanelsField.presets
import com.bylazar.field.Style
import com.pedropathing.follower.Follower
import com.pedropathing.geometry.*
import com.pedropathing.math.*
import com.pedropathing.paths.*
import com.pedropathing.util.*
import java.lang.Double


class Drawing {
    val ROBOT_RADIUS: kotlin.Double = 8.125
    private val panelsField = field

    private val robotLook = Style(
        "", "#3F51B5", 0.75
    )
    private val historyLook = Style(
        "", "#4CAF50", 0.75
    )

    fun init() {
        panelsField.setOffsets(presets.PEDRO_PATHING)
    }

    fun drawDebug(follower: Follower) {
        if (follower.currentPath != null) {
            drawPath(follower.currentPath, robotLook)
            val closestPoint =
                follower.getPointFromPath(follower.currentPath.closestPointTValue)
            drawRobot(
                Pose(
                    closestPoint.x,
                    closestPoint.y,
                    follower.currentPath
                        .getHeadingGoal(follower.currentPath.closestPointTValue)
                ), robotLook
            )
        }
        drawPoseHistory(follower.poseHistory, historyLook)
        drawRobot(follower.pose, historyLook)

        sendPacket()
    }

    @JvmOverloads
    fun drawRobot(pose: Pose?, style: Style = robotLook) {
        if (pose == null || Double.isNaN(pose.x) || Double.isNaN(pose.y) || Double.isNaN(
                pose.heading
            )
        ) {
            return
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(pose.x, pose.y)
        panelsField.circle(ROBOT_RADIUS)

        val v = pose.headingAsUnitVector
        v.magnitude = v.magnitude * ROBOT_RADIUS
        val x1 = pose.x + v.xComponent / 2
        val y1 = pose.y + v.yComponent / 2
        val x2 = pose.x + v.xComponent
        val y2 = pose.y + v.yComponent

        panelsField.setStyle(style)
        panelsField.moveCursor(x1, y1)
        panelsField.line(x2, y2)
    }

    fun drawPath(path: Path, style: Style) {
        val points = path.panelsDrawingPoints

        for (i in points[0]!!.indices) {
            for (j in points.indices) {
                if (Double.isNaN(points[j]!![i])) {
                    points[j]!![i] = 0.0
                }
            }
        }

        panelsField.setStyle(style)
        panelsField.moveCursor(points[0]!![0], points[0]!![1])
        panelsField.line(points[1]!![0], points[1]!![1])
    }

    fun drawPath(pathChain: PathChain, style: Style) {
        for (i in 0..<pathChain.size()) {
            drawPath(pathChain.getPath(i), style)
        }
    }

    @JvmOverloads
    fun drawPoseHistory(poseTracker: PoseHistory, style: Style = historyLook) {
        panelsField.setStyle(style)

        val size = poseTracker.xPositionsArray.size
        for (i in 0..<size - 1) {
            panelsField.moveCursor(
                poseTracker.xPositionsArray[i],
                poseTracker.yPositionsArray[i]
            )
            panelsField.line(
                poseTracker.xPositionsArray[i + 1],
                poseTracker.yPositionsArray[i + 1]
            )
        }
    }

    fun sendPacket() {
        panelsField.update()
    }
}