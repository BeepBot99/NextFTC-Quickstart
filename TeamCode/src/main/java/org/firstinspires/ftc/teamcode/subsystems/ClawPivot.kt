package org.firstinspires.ftc.teamcode.subsystems

import com.acmerobotics.dashboard.config.Config
import com.qualcomm.robotcore.hardware.Servo
import com.rowanmcalpin.nextftc.core.Subsystem
import com.rowanmcalpin.nextftc.core.command.Command
import com.rowanmcalpin.nextftc.ftc.OpModeData
import com.rowanmcalpin.nextftc.ftc.hardware.ServoToPosition

@Config
object ClawPivot: Subsystem() {
    lateinit var servo: Servo

    val name = "claw_servo"

    @JvmField
    var downPosition = 0.9

    @JvmField
    var upPosition = 0.2

    val toDown: Command
        get() = ServoToPosition(servo, // SERVO TO MOVE
            downPosition, // POSITION TO MOVE TO
            this)  // IMPLEMENTED SUBSYSTEM

    val toUp: Command
        get() = ServoToPosition(servo, // SERVO TO MOVE
            upPosition, // POSITION TO MOVE TO
            this) // IMPLEMENTED SUBSYSTEM

    override fun initialize() {
        servo = OpModeData.hardwareMap.get(Servo::class.java, name)
    }
}