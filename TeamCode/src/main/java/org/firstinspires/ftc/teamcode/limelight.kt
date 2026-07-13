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
    
    enum class LimelightState {
        ON, OFF
    }



}