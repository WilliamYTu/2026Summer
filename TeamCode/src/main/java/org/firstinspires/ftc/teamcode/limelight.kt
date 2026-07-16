package org.firstinspires.ftc.teamcode

import com.bylazar.panels.Panels
import com.bylazar.configurables.annotations.Configurable
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry
import java.lang.Math.toRadians

import kotlin.math.tan
import kotlin.math.sqrt
// import kotlin.math.toRadians

@Configurable
class Limelight(private val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    private lateinit var limelight: Limelight3A
    private lateinit var imu: IMU
    @JvmField var servoPos = 500.0
    private  val CAMERA_HEIGHT_INCHES = 8.0
    private  val CAMERA_ANGLE_DEGREES = 0.0
    private  val BALL_HEIGHT_INCHES = 2.0     // measure from center of the ball to ground
    private  val SAFETY = -999.0
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

        imu = hardwareMap.get(IMU::class.java, "imu")

        val revHubOrientationOnRobot = RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
            RevHubOrientationOnRobot.UsbFacingDirection.UP
        )

        imu.initialize(IMU.Parameters(revHubOrientationOnRobot))
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

    fun switchPipeline(pipeline: Int) {
        limelight.pipelineSwitch(pipeline)
    }



    data class DetectedBall(
        val tx: Double,
        val ty: Double,
        val forwardInches: Double,
        val lateralInches: Double,
        val straightLineDistanceInches: Double
    )

    data class BallPosition(val forward: Double, val lateral: Double, val distance: Double)

    fun displacementFromAngles(): List<BallPosition> {
        val result: LLResult? = limelight.getLatestResult()

        if (result != null && result.isValid) {
            val pythonOutput = result.pythonOutput

            val points = pythonOutput.take(8)
                .chunked(2)
                .takeWhile { pair -> pair.size == 2 && -999.0 !in pair }

            val ballPositions: List<BallPosition> = points.map { pair ->
                val tx = pair[0]
                val ty = pair[1]

                val angleToTargetRadians = toRadians(CAMERA_ANGLE_DEGREES + ty)
                val forward = (BALL_HEIGHT_INCHES - CAMERA_HEIGHT_INCHES) / tan(angleToTargetRadians)
                val lateral = forward * tan(toRadians(tx))
                val distance = sqrt(forward * forward + lateral * lateral)
                BallPosition(forward, lateral, distance)
            }
            return ballPositions
        }
        return emptyList()
    }



    enum class LimelightState {
        ON, OFF
    }



}