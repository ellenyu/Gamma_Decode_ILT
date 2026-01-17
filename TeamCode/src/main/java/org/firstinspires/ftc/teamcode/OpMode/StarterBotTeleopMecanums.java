package org.firstinspires.ftc.teamcode.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    private final StarterBotTeleopMecanums starterBotTeleopMecanums;
    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1;

    final double INTAKE_FORWARD_VELOCITY = 0.5;
    final double INTAKE_BACKWARD_VELOCITY = -0.5;

    boolean forward = true;

    final double BUMPER_FEED_TIME = 0.55;
    final double BUMPER_FEED_POWER = -0.3;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;

    ElapsedTime feederTimer = new ElapsedTime();

    public StarterBotTeleopMecanums(StarterBotTeleopMecanums starterBotTeleopMecanums) {
        this.starterBotTeleopMecanums = starterBotTeleopMecanums;
    }

    // 🔹 UPDATED STATE MACHINE


    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorbr");
        intake = hardwareMap.get(DcMotor.class, "intake");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        //   leftFeeder.setPower(STOP_SPEED);
        //   rightFeeder.setPower(STOP_SPEED);


        telemetry.addData("Status", "Initialized");
    }


    @Override
    public void loop() {

        starterBotTeleopMecanums.mecanumDrive(
                gamepad1.left_stick_y,
                gamepad1.left_stick_x,
                -gamepad1.right_stick_x
        );

        // Y starts intake and keeps it running
//        if (gamepad1.y) {
//            hogback.setVelocity(HOGBACK_TARGET_VELOCITY);
//        }

        // A stops hogback
//        if (gamepad1.a) {
//            hogback.setVelocity(STOP_SPEED);
//        }

        // Right trigger fires 3 shots
        if (gamepad2.right_bumper) {
            intake.setPower(0);
            forward = true;

        }

        if (gamepad2.right_trigger > 0.5) {
            if (forward) {
                forward = false;
                intake.setPower(INTAKE_FORWARD_VELOCITY);
            } else {
                forward = true;
                intake.setPower(INTAKE_BACKWARD_VELOCITY);
            }

        }
    }

    void mecanumDrive(double forward, double strafe, double rotate) {
        double denominator = Math.max(
                Math.abs(forward) + Math.abs(strafe) + Math.abs(rotate),
                1
        );

        leftFrontDrive.setPower((forward + strafe + rotate) / denominator);
        rightFrontDrive.setPower((forward - strafe - rotate) / denominator);
        leftBackDrive.setPower((forward - strafe + rotate) / denominator);
        rightBackDrive.setPower((forward + strafe - rotate) / denominator);
    }


    // Right bumper jogs feeders for 0.1s
//        if (gamepad1.right_bumper && launchState == LaunchState.IDLE) {
//            feederTimer.reset();
//            leftFeeder.setPower(BUMPER_FEED_POWER);
//            rightFeeder.setPower(BUMPER_FEED_POWER);
//            launchState = LaunchState.JOG_FEEDER;
//        }

    // launch(maxShots > 0);

//        telemetry.addData("State", launchState);
//        telemetry.addData("Hogback Velocity", hogback.getVelocity());
//


}
