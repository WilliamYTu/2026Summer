package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcore.external.Telemetry

class Limelight(private val hardwareMap: HardwareMap, val telemetry: Telemetry) {
    private lateinit var limelight: Limelight3A
    private lateinit var imu: IMU
    // private val k = 75.78304131
    var state = LimelightState.OFF



    fun initLimelight() {
        limelight = hardwareMap.get(Limelight3A::class.java, "Ethernet Device")

        val limelightServo = (hardwareMap["limelightServo"] as ServoImplEx).apply {
            direction = Servo.Direction.FORWARD
            // pwmRange = PwmControl.PwmRange(520.0, 2480.0)
        }


        fun limelightServoPos(pos: Double){
            limelightServo.position = pos
        }

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

    fun detectBalls() {
        val result: LLResult? = limelight.latestResult

        if (result != null && result.isValid) {
            // ---- Channel 1: built-in angle to the CLOSEST ball (either color) ----
            val tx = result.tx  // degrees, + = right of crosshair
            val ty = result.ty  // degrees, + = above crosshair

            // ---- Channel 2: your custom array ----
            val py = result.pythonOutput  // DoubleArray?, matches llpython
            if (py != null && py.size >= 8) {
                val totalBalls   = py[0].toInt()
                val purpleCount  = py[1].toInt()
                val purpleX      = py[2]  // pixel coords, NOT degrees
                val purpleY      = py[3]
                val greenCount   = py[4].toInt()
                val greenX       = py[5]
                val greenY       = py[6]

                telemetry.addData("Total balls", totalBalls)
                telemetry.addData("Purple", "count=$purpleCount x=$purpleX y=$purpleY")
                telemetry.addData("Green", "count=$greenCount x=$greenX y=$greenY")
            }

            telemetry.addData("Closest ball tx", tx)
            telemetry.addData("Closest ball ty", ty)
        } else {
            telemetry.addData("Limelight", "no valid result")
        }

        telemetry.update()
    }

    enum class LimelightState {
        ON, OFF
    }



}