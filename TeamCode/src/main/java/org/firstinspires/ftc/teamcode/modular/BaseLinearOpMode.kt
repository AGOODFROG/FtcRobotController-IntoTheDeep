package org.firstinspires.ftc.teamcode.modular

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.TouchSensor
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver

@Suppress("MemberVisibilityCanBePrivate")
abstract class BaseLinearOpMode : LinearOpMode() {
    protected lateinit var leftBack: DcMotorEx
    protected lateinit var rightBack: DcMotorEx
    protected lateinit var rightFront: DcMotorEx
    protected lateinit var leftFront: DcMotorEx
    protected lateinit var allMotors: Array<DcMotorEx>
    protected lateinit var odometry: GoBildaPinpointDriver
    protected lateinit var spinner: Spinner
    protected lateinit var bucket: Servo
    protected lateinit var switch: TouchSensor
    protected lateinit var elevator: DcMotorEx
    protected lateinit var arm: DcMotorEx
    protected lateinit var ratchet: ServoWrapper
    protected lateinit var hooks: ServoWrapper

    protected fun initHardware() {
        this.leftBack = this.hardwareMap["left_back"] as DcMotorEx
        this.rightBack = this.hardwareMap["right_back"] as DcMotorEx
        this.rightFront = this.hardwareMap["right_front"] as DcMotorEx
        this.leftFront = this.hardwareMap["left_front"] as DcMotorEx

        this.leftFront.direction = DcMotorSimple.Direction.REVERSE
        this.rightFront.direction = DcMotorSimple.Direction.FORWARD
        this.leftBack.direction = DcMotorSimple.Direction.REVERSE
        this.rightBack.direction = DcMotorSimple.Direction.FORWARD

        this.allMotors = arrayOf(leftFront, rightFront, leftBack, rightBack)

        this.odometry = this.hardwareMap["odometry"] as GoBildaPinpointDriver
        this.odometry.setOffsets(95.0, 0.0)
        this.odometry.setEncoderResolution(37.25135125)
        this.odometry.setEncoderDirections(
            GoBildaPinpointDriver.EncoderDirection.FORWARD,
            GoBildaPinpointDriver.EncoderDirection.REVERSED
        )
        odometry.resetPosAndIMU()

        arm = this.hardwareMap["arm"] as DcMotorEx
        arm.direction = DcMotorSimple.Direction.REVERSE
        arm.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        arm.zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE

        elevator = this.hardwareMap["elevator"] as DcMotorEx
        elevator.direction = DcMotorSimple.Direction.REVERSE
        elevator.mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER

        ratchet = ServoWrapper(this.hardwareMap.servo["ratchet"], 0.1, 0.0)
        ratchet.disengage()

        hooks = ServoWrapper(this.hardwareMap.servo["hooks"], 0.6, 0.2)
        hooks.disengage()
        hooks.disablePwm()

        val leftSpinner = this.hardwareMap["left_spinner"] as CRServo
        leftSpinner.direction = DcMotorSimple.Direction.REVERSE
        val rightSpinner = this.hardwareMap["right_spinner"] as CRServo
        spinner = Spinner(leftSpinner, rightSpinner)

        bucket = this.hardwareMap["bucket"] as Servo

        switch = this.hardwareMap["touch_sensor"] as TouchSensor
    }
}
