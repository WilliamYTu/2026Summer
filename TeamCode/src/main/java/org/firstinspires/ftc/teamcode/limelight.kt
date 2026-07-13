package org.firstinspires.ftc.teamcode

import com.pedropathing.geometry.Pose
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.Telemetry
import kotlin.math.abs
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

class Limelight(private val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    private lateinit var limelight: Limelight3A
    private lateinit var imu: IMU
    private val k = 75.78304131
    var state = LimelightState.OFF;



    fun initLimelight() {
        limelight = hardwareMap.get(Limelight3A::class.java, "Ethernet Device")

        limelight.pipelineSwitch(1)

        imu = hardwareMap.get(IMU::class.java, "imu")

        val revHubOrientationOnRobot = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )

        imu.initialize(IMU.Parameters(revHubOrientationOnRobot))
    }

    fun start(){
        state = LimelightState.ON
        limelight.start()
    }

    fun stop(){
        state = LimelightState.OFF
        limelight.stop()
    }

    fun getTargetOffsetAngles(): Pair<Double, Double>? {
        val llResult = limelight.latestResult
        return if (llResult != null && llResult.isValid) {
            val yaw = Math.toRadians(llResult.tx)
            val pitch = Math.toRadians(llResult.ty)
            Pair(yaw, pitch)
        } else null
    }
    fun switchPipeline(pipeline: Int) {
        limelight.pipelineSwitch(pipeline)
    }

    fun isColorDetected(className: String? = null): Boolean {
        val llResult = limelight.latestResult
        if (llResult == null || !llResult.isValid) return false

        if (className != null) {
            return llResult.detectorResults.any { it.className.equals(className, ignoreCase = true) }
        }

        return llResult.colorResults.isNotEmpty() || llResult.detectorResults.isNotEmpty()
    }

    fun getColorTargetAngles(): Pair<Double, Double>? {
        val llResult = limelight.latestResult
        if (llResult != null && llResult.isValid) {
            if (llResult.colorResults.isNotEmpty()) {
                val target = llResult.colorResults[0]
                return Pair(target.targetXDegrees, target.targetYDegrees)
            } else if (llResult.detectorResults.isNotEmpty()) {
                val target = llResult.detectorResults[0]
                // Note: DetectorResult also has targetXDegrees and targetYDegrees in the API
                return Pair(target.targetXDegrees, target.targetYDegrees)
            }
        }
        return null
    }

    fun getDistanceFromTag(): Pair<Double, Double>? { // uses Ta, pretty unreliable
        val llResult = limelight.latestResult
        // cos(Ty) = linearDistance / hypotenuse
        // hypotenuse = linearDistance / cos(Ty)
        if (llResult != null && llResult.isValid) {
            val Ta = llResult.ta
            val Ty = llResult.ty
            val linearDistance  = k / sqrt(Ta)
            val angularDistance = abs(linearDistance / cos(Ty))
            return Pair(angularDistance, linearDistance)
            // return angularDistance
        } else {
            return null
        }
    }

    fun getLocalTagPose(): Distances { // uses built-in pose, for all intensive purposes use this one
        val llResult = limelight.latestResult

        if (llResult == null || !llResult.isValid || llResult.fiducialResults.isEmpty()) {
            return Distances(0.0, 0.0,0.0,0.0,0.0,0.0,0.0,0.0)
        }

        var heading = 0.0
        var xDist = 0.0
        var yDist = 0.0
        var zDist = 0.0
        var extraDistOne = 0.0
        var extraDistTwo = 0.0
        var horizontalDist = 0.0
        var slantDist = 0.0

        if (llResult.isValid && llResult.fiducialResults.isNotEmpty()) {
            val tag = llResult.fiducialResults[0]
            val pose = tag.robotPoseTargetSpace
            // val pose  = llResult.botpose_MT2

            val x = pose.position.x
            val y = pose.position.y
            val z = pose.position.z
            heading = pose.orientation.yaw

            // convert from meters to inches
            xDist = 39.3701 * x
            yDist = 39.3701 * y
            zDist = 39.3701 * z
            extraDistOne = 39.3701 * sqrt(x * x + y * y)
            extraDistTwo = 39.3701 * sqrt(y * y + z * z)
            horizontalDist = 39.3701 * sqrt(x * x + z * z)
            slantDist = 39.3701 * sqrt(x * x + y * y + z * z)
        }

        return Distances(heading, xDist, yDist, zDist, extraDistOne, extraDistTwo, horizontalDist, slantDist)

    }


    data class Distances(
        val heading: Double,
        val xDist: Double,
        val yDist: Double,
        val zDist: Double,
        val extraDistOne: Double,
        val extraDistTwo: Double,
        val horizontalDist: Double,
        val slantDist: Double
    )

    fun latestResultValid(): Boolean{
        val llResult = limelight.latestResult
        if (llResult.isValid){
            return true
        } else {
            return false
        }
    }

    fun getGlobalPose(color: String): Pose {
        val llResult = limelight.latestResult
        val conversion = 39.37007874
        val blueID = 20
        val redID = 24
        if (llResult == null || !llResult.isValid || llResult.fiducialResults.isEmpty()) {
            return Pose(0.0, 0.0,0.0)
        }
        val pose = llResult.fiducialResults[0].targetPoseCameraSpace
        val tagId = llResult.fiducialResults[0].fiducialId
        val heading = -pose.orientation.yaw
        // axes remapping
        val x = conversion * pose.position.z
        val y = -1 * conversion * pose.position.x
        val z = -1 * conversion * pose.position.y
        val tagPoseRelativeToCamera = Pose(x, y, heading) // tag relative to camera after remapping axes

        var cameraPoseOnField = when (tagId) { // conversion to find camera position on field
            blueID -> multiplyPoses(BLUETAGPOSE, invertPose(tagPoseRelativeToCamera))
            redID -> multiplyPoses(REDTAGPOSE, invertPose(tagPoseRelativeToCamera))
            else ->  multiplyPoses(REDTAGPOSE, invertPose(tagPoseRelativeToCamera))
        }

        //factor in offset, camera isn't at center of robot
        val robotPose = multiplyPoses(cameraPoseOnField, invertPose(cameraOffset))

        // make sure taking readings from right aprilTag
        var targetTag = when (color){
            "blue" -> 20
            "red" -> 24
            else -> 0
        }

        return (if (tagId == targetTag) robotPose else Pose(0.0,0.0,0.0))
    }



    fun invertPose(p: Pose): Pose {
        val cos = cos(p.heading)
        val sin = sin(p.heading)

        val xInv = -(cos * p.x + sin * p.y)
        val yInv = sin * p.x - cos * p.y

        return Pose(xInv, yInv, -p.heading)
    }

    fun multiplyPoses(a: Pose, b: Pose): Pose {
        val cos = Math.cos(a.heading)
        val sin = Math.sin(a.heading)

        val x = a.x + b.x * cos - b.y * sin
        val y = a.y + b.x * sin + b.y * cos
        val heading = a.heading + b.heading

        return Pose(x, y, heading)
    }
    fun getTagID(allowedIds: List<Int>? = null): Int? {
        val orientation = imu.robotYawPitchRollAngles
        limelight.updateRobotOrientation(orientation.yaw)

        val llResult = limelight.latestResult

        if (llResult != null && llResult.isValid && llResult.fiducialResults.isNotEmpty()) {
            if (allowedIds != null) {
                for (detection in llResult.fiducialResults) {
                    val tagId = detection.fiducialId
                    if (allowedIds.contains(tagId)) {
                        return tagId
                    }
                }
                return null
            } else {
                return llResult.fiducialResults[0].fiducialId
            }
        }

        return null
    }

    companion object {
        val REDTAGPOSE = Pose(128.0, 132.0, 130.0)

        val BLUETAGPOSE = Pose(16.0, 132.0, 50.0)

        val cameraOffset = Pose(7.0,0.0,0.0)

        @JvmField
        var limelightWeightX = 1

        @JvmField
        var limelightWeightY = 1
    }

    enum class LimelightState {
        ON, LOCALIZED, OFF
    }



}