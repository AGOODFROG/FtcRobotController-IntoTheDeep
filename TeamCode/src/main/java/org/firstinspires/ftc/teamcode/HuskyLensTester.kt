package org.firstinspires.ftc.teamcode

import com.qualcomm.hardware.dfrobot.HuskyLens
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.Telemetry
import org.firstinspires.ftc.robotcore.internal.system.Deadline
import org.firstinspires.ftc.teamcode.modular.GamepadButton
import org.firstinspires.ftc.teamcode.modular.GamepadState
import java.util.concurrent.TimeUnit
import org.firstinspires.ftc.teamcode.modular.BaseLinearOpMode

/**
 * Sample opmode (and reference implementation) for testing out HuskeyLens
 * Requires HuskeyLens (duh), and optionally controller for switching algorithms.
 * Button mapping:
 * +=============+============+=======================+
 * | Xbox button | PS5 button |    HuskeyLens mode    |
 * +=============+============+=======================+
 * | A           | Cross      | Object recognition    |
 * +-------------+------------+-----------------------+
 * | B           | Circle     | Tag recognition       |
 * +-------------+------------+-----------------------+
 * | X           | Square     | Color recognition     |
 * +-------------+------------+-----------------------+
 * | Y           | Triangle   | Object classification |
 * +-------------+------------+-----------------------+
 * It pipes the coordinates of all of the box boundaries (or lines) to telemetry once started.
 *
 * Once we figure out what we'll use the HuskeyLens for, I'll write a class to easily use it
 */

@TeleOp(name = "HuskyLens Tester")
class HuskyLensTester : BaseLinearOpMode() {
    // Define all hardware (gamepad and HuskyLens)
    private lateinit var gp1: GamepadState
    private lateinit var huskyLens: HuskyLens

    // So having a ratelimit is apparently important, "to make it easier to read"
    private var readPeriod: Long = 1
    private var rateLimit: Deadline = Deadline(readPeriod, TimeUnit.SECONDS)

    // Updates active camera algorithm when called
    class LensMode(private val huskyLens: HuskyLens, private val telemetry: Telemetry) {
        fun tagRecognition() {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION)
            telemetry.addData(">>", "Set HuskyLens algorithm to tag recognition.")
            telemetry.update()
        }

        fun colorRecognition() {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.COLOR_RECOGNITION)
            telemetry.addData(">>", "Set HuskyLens algorithm to color recognition.")
            telemetry.update()
        }

        fun objectRecognition() {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_RECOGNITION)
            telemetry.addData(">>", "Set HuskyLens algorithm to object recognition.")
            telemetry.update()
        }

        fun objectClassification() {
            huskyLens.selectAlgorithm(HuskyLens.Algorithm.OBJECT_CLASSIFICATION)
            telemetry.addData(">>", "Set HuskyLens algorithm to object classification.")
            telemetry.update()
        }
    }

    override fun runOpMode() {
        huskyLens = hardwareMap.get(HuskyLens::class.java, "huskylens")

        rateLimit.expire()

        gp1 = GamepadState(gamepad1)

        LensMode(
            huskyLens,
            telemetry
        ).colorRecognition() // Sets the default mode. Most likely we'll use color recognition to detect game objects.

        // Proof of life check
        if (huskyLens.knock()) {
            telemetry.addData(
                ">>",
                "Connected to ${huskyLens.deviceName}, press start to continue."
            )
            telemetry.update()
        } else {
            telemetry.addData(">>", "Could not communicate with ${huskyLens.deviceName}")
            telemetry.update()
        }

        this.waitForStart() // Done initialization process

        while (this.opModeIsActive()) {
            if (!rateLimit.hasExpired()) {
                continue
            }
            rateLimit.reset()
            this.gp1.cycle()

            // ALlow selection of camera algorithm by controller buttons.
            // You may need to hold it for a second to work.
            // See table in docstring for controls.
            if (GamepadButton(gp1, Gamepad::circle).isPressed) {
                LensMode(huskyLens, telemetry).tagRecognition()
            } else if (GamepadButton(gp1, Gamepad::square).isPressed) {
                LensMode(huskyLens, telemetry).colorRecognition()
            } else if (GamepadButton(gp1, Gamepad::cross).isPressed) {
                LensMode(huskyLens, telemetry).objectRecognition()
            } else if (GamepadButton(gp1, Gamepad::triangle).isPressed) {
                LensMode(huskyLens, telemetry).objectClassification()
            }

            val blocks: Array<HuskyLens.Block> = huskyLens.blocks()
            val arrows: Array<HuskyLens.Arrow> = huskyLens.arrows()


            // All algorithms except for LINE_TRACKING return a list of blocks (outlines)
            // of the detected objects, as well as an ID number. Returns an empty array if nothing
            // is detected.
            // Here, the full block data is parsed and printed.
            for (block in blocks) {
                telemetry.addData("Detected block: ", block.id.toString())
                telemetry.addData("Block size: ", (block.width * block.height).toString())
                telemetry.addData("Block edges: ", (block.left and block.top).toString())
                telemetry.addData("Box center: ", "(${block.x}, ${block.y})")
                telemetry.addData("Raw block array: ", block.toString())
            }

            for (arrow in arrows) {
                telemetry.addData("Detected arrow: ", arrow.id.toString())
                telemetry.addData("Arrow origin: ", "(${arrow.x_origin}, ${arrow.y_origin})")
                telemetry.addData("Arrow target: ", "(${arrow.x_target}, ${arrow.x_target})")
            }

            telemetry.update()
        }
    }
}
