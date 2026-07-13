package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.limelightvision.Limelight3A
import com.qualcomm.hardware.limelightvision.LLResult
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp

@TeleOp(name = "Ball Detector")
class BallDetectorOpMode : LinearOpMode() {

    private lateinit var limelight: Limelight3A

    override fun runOpMode() {
        limelight = hardwareMap.get(Limelight3A::class.java, "limelight")
        limelight.pipelineSwitch(0)   // whichever pipeline slot holds your script
        limelight.setPollRateHz(100)  // poll rate — matches your loop speed
        limelight.start()             // MUST be called before latestResult works

        waitForStart()

        while (opModeIsActive()) {
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

        limelight.stop()
    }
}