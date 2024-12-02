package org.firstinspires.ftc.teamcode

import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Gamepad
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import org.firstinspires.ftc.teamcode.modular.BaseLinearOpMode
import org.firstinspires.ftc.teamcode.modular.GamepadButton
import org.firstinspires.ftc.teamcode.modular.GamepadState
import org.firstinspires.ftc.teamcode.modular.ToggleableState
import java.util.Locale
import kotlin.math.abs

@TeleOp(name = "DriveTrain")
// @Disabled
class DriveTrain : BaseLinearOpMode() {
    // kotlin does not do numeric type promotion, if the 3rd arg is just "1" than T cannot be inferred
    private var power = ToggleableState(2, 0.33, 0.67, 1.0)
    private lateinit var gp1: GamepadState

    override fun runOpMode() {/* Initialization */
        telemetry.msTransmissionInterval = 100

        gp1 = GamepadState(gamepad1)

        val toggleButtonMap = mapOf(
            GamepadButton(gp1, Gamepad::left_bumper) to power::left,
            GamepadButton(gp1, Gamepad::right_bumper) to power::right
        )

        this.initDriveTrain()

        /* End Initialization */
        this.waitForStart()

        while (this.opModeIsActive()) {
            this.gp1.cycle()
            this.odometry.update()
            val pos = this.odometry.position;
            val data = String.format(
                Locale.US,
                "x: %.3f, y: %.3f, h: %.3f",
                pos.getX(DistanceUnit.MM),
                pos.getY(DistanceUnit.MM),
                pos.getHeading(AngleUnit.DEGREES)
            );
            telemetry.addData("pos", data);

            toggleButtonMap.forEach { it.key.ifIsToggled(it.value) }

            /* Calculates motor power in accordance with the allMotors array
               and formulas found here: https://github.com/brandon-gong/ftc-mecanum
             */
            val turnPower = -this.gp1.current.right_trigger + this.gp1.current.left_trigger

            val motorPower = arrayOf(
                this.gp1.current.left_stick_x - this.gp1.current.left_stick_y - turnPower,
                -this.gp1.current.left_stick_x - this.gp1.current.left_stick_y + turnPower,
                -this.gp1.current.left_stick_x - this.gp1.current.left_stick_y - turnPower,
                this.gp1.current.left_stick_x - this.gp1.current.left_stick_y + turnPower,
            )

            // Magnitude of the maximum value, not velocity
            val max = abs(motorPower.maxBy { abs(it) })

            // Normalize if greater the max is greater than 1
            if (max > 1) motorPower.forEachIndexed { i, _ -> motorPower[i] /= max }

            // Update the motors with the proper power
            this.allMotors.forEachIndexed { i, m ->
                m.power = motorPower[i].toDouble() * power.value
            }

            telemetry.update()
        }

    }
}
