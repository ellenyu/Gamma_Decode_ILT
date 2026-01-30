package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;

@Autonomous(name = "StarterBotAuto3Ball", group = "StarterBot")
@Disabled
public class StarterBotAuto extends LinearOpMode {

    // Drive motors
    private DcMotor motorfl, motorfr, motorbl, motorbr;

    // Shooter motors/servos
    private DcMotor hogback;
    private CRServo flyWheell, flyWheelr;

    // ----------- Shooter Constants -----------
    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = 1.0; // full power for servos and motors
    final double HOGBACK_TARGET_VELOCITY = 1900; // for reference
    final double HOGBACK_MIN_VELOCITY = 1800;    // for reference

    @Override
    public void runOpMode() throws InterruptedException {

        // ----------- Hardware Map -----------
        motorfl = hardwareMap.get(DcMotor.class, "motorfl");
        motorfr = hardwareMap.get(DcMotor.class, "motorfr");
        motorbl = hardwareMap.get(DcMotor.class, "motorbl");
        motorbr = hardwareMap.get(DcMotor.class, "motorbr");

        hogback = hardwareMap.get(DcMotor.class, "hogback");
        flyWheell = hardwareMap.get(CRServo.class, "flyWheell");
        flyWheelr = hardwareMap.get(CRServo.class, "flyWheelr");

        // ----------- Motor Directions -----------
        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        motorfr.setDirection(DcMotor.Direction.FORWARD);
        motorbr.setDirection(DcMotor.Direction.FORWARD);

        hogback.setDirection(DcMotor.Direction.REVERSE);
        flyWheell.setDirection(CRServo.Direction.FORWARD);
        flyWheelr.setDirection(CRServo.Direction.REVERSE);

        // ----------- Initial States -----------
        motorfl.setPower(0);
        motorfr.setPower(0);
        motorbl.setPower(0);
        motorbr.setPower(0);

        flyWheell.setPower(0);
        flyWheelr.setPower(0);
        hogback.setPower(0);

        telemetry.addLine("Initialized - READY");
        telemetry.update();

        waitForStart();

        // ----------- Shoot Routine -----------
        // Spin up hogback motor
        hogback.setPower(FULL_SPEED);
        sleep(500); // wait for motor to reach near-target speed

        // Start flywheels
        flyWheell.setPower(FULL_SPEED);
        flyWheelr.setPower(FULL_SPEED);

        // Feed balls
        sleep((long)(FEED_TIME_SECONDS * 1000)); // 0.5 seconds

        // Stop all shooter motors
        flyWheell.setPower(STOP_SPEED);
        flyWheelr.setPower(STOP_SPEED);
        hogback.setPower(STOP_SPEED);

        // ----------- Drive Backward -----------
        motorfl.setPower(-0.4);
        motorfr.setPower(-0.4);
        motorbl.setPower(-0.4);
        motorbr.setPower(-0.4);
        sleep(4000); // move backward 4 seconds

        // Stop motors
        motorfl.setPower(0);
        motorfr.setPower(0);
        motorbl.setPower(0);
        motorbr.setPower(0);

        // Loop to keep opmode alive
        while (opModeIsActive()) {
            idle();
        }
    }
}
