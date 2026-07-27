package org.firstinspires.ftc.teamcode

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
    @JvmField var servoPos = 0.2
    private  val cameraHeightInches = 11.0
    private  val cameraAngleDegrees = -16.0
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
                val forward = (ballHeightInches - cameraHeightInches) / tan(angleToTargetRadians)
                val lateral = forward * tan(toRadians(tx))
                val distance = sqrt(forward * forward + lateral * lateral)
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



    enum class LimelightState {
        ON, OFF
    }



}