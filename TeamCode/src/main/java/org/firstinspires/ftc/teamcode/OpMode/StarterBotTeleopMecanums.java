package org.firstinspires.ftc.teamcode.OpMode;

import static com.qualcomm.hardware.bosch.BNO055IMU.SystemStatus.IDLE;
import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;
import com.qualcomm.robotcore.hardware.CRServo;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.motors.RevRobotics20HdHexMotor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    RobotHardware robot = new RobotHardware();

    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1;

    final double INTAKE_FORWARD_VELOCITY = 0.8;
    final double INTAKE_BACKWARD_VELOCITY = -0.6;

    boolean forward = true;

    final double BUMPER_FEED_TIME = 0.55;
    final double BUMPER_FEED_POWER = -0.3;

    final double HOGBACK_TARGET_VELOCITY = 1800;
    final double HOGBACK_MIN_VELOCITY = 1700;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;

    private CRServo pusher0;
    private CRServo pusher1;

    private Servo sqbeam;

    private DcMotorEx hogback;
    private Limelight3A limelight;

    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    ElapsedTime feederTimer = new ElapsedTime();

    int maxShots = 0;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
    }
    // 🔹 UPDATED STATE MACHINE

    private LaunchState launchState;

    @Override
    public void init() {

        leftFrontDrive = hardwareMap.get(DcMotor.class, "motorfl");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "motorfr");
        leftBackDrive = hardwareMap.get(DcMotor.class, "motorbl");
        rightBackDrive = hardwareMap.get(DcMotor.class, "motorbr");
        intake = hardwareMap.get(DcMotor.class, "intake");
        hogback = hardwareMap.get(DcMotorEx.class, "hogback");


        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setDirection(DcMotor.Direction.REVERSE);

        pusher0 = hardwareMap.get(CRServo.class, "pusher0");
        pusher1 = hardwareMap.get(CRServo.class, "pusher1");
        launchState = LaunchState.IDLE;
        sqbeam = hardwareMap.get(Servo.class, "sqbeam");


        leftFrontDrive.setZeroPowerBehavior(BRAKE);
        rightFrontDrive.setZeroPowerBehavior(BRAKE);
        leftBackDrive.setZeroPowerBehavior(BRAKE);
        rightBackDrive.setZeroPowerBehavior(BRAKE);
        intake.setZeroPowerBehavior(BRAKE);

        // leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);
        //   leftFeeder.setPower(STOP_SPEED);
        //   rightFeeder.setPower(STOP_SPEED);


        telemetry.addData("Status", "Initialized");


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        imu = hardwareMap.get(IMU.class, "imu");

        // Adjust these two settings to match how your Hub is mounted!
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));



        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCameraResolution(new Size(640, 480));
//        builder.addProcessor(aprilTagProcessor);
//
//        visionPortal = builder.build();

// end


    }


    @Override
    public void loop() {

        double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
        double vertical = gamepad1.right_stick_y * 0.6;
        double turn = -1.0 * gamepad1.left_stick_x * 0.6;

        //double tx = LimelightHelpers.getTX("limelight");
        // boolean hasTarget = LimelightHelpers.hasTarget("limelight");

        double flPower = vertical + turn + horizontal;
        double frPower = vertical - turn - horizontal;
        double blPower = vertical + turn - horizontal;
        double brPower = vertical - turn + horizontal;
        double scaling = Math.max(1.0,
                Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                        Math.max(Math.abs(blPower), Math.abs(brPower))));
        flPower = flPower / scaling;
        frPower = frPower / scaling;
        blPower = blPower / scaling;
        brPower = brPower / scaling;

        if (leftFrontDrive == null || rightFrontDrive == null || leftBackDrive == null || rightBackDrive == null) {
            telemetry.addData("DriveMotorNull", String.format("lf:%b rf:%b lb:%b rb:%b",
                    leftFrontDrive == null, rightFrontDrive == null, leftBackDrive == null, rightBackDrive == null));
            telemetry.update();
        } else {
            leftFrontDrive.setPower(flPower);
            rightFrontDrive.setPower(frPower);
            leftBackDrive.setPower(blPower);
            rightBackDrive.setPower(brPower);
        }



//limelight
        // --- Limelight Logic (Replaces your while loop) ---
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", botpose !=null ? botpose.toString(): "null");
        }

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            telemetry.addData("Python output:", pythonOutputs[0]);
        }

        // First, tell Limelight which way your robot is facing
        double robotYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

//drive chain
        if (gamepad2.y) {
            leftFrontDrive.setZeroPowerBehavior(BRAKE);
            rightFrontDrive.setZeroPowerBehavior(BRAKE);
            leftBackDrive.setZeroPowerBehavior(BRAKE);
            rightBackDrive.setZeroPowerBehavior(BRAKE);
            hogback.setPower(HOGBACK_TARGET_VELOCITY);
        }

//hogback
        if (gamepad2.a) {
            hogback.setVelocity(STOP_SPEED);
        }


//intake
            if (gamepad2.right_bumper) {
                telemetry.addLine("intake stop");
                forward = true;
                intake.setPower(0);

            }

            if (gamepad2.left_bumper) {
                telemetry.addLine("left bumper is pressed");
                if (forward) {
                    telemetry.addData("intake forward:", INTAKE_FORWARD_VELOCITY);
                    forward = false;
                    intake.setPower(INTAKE_FORWARD_VELOCITY);
                } else {
                    telemetry.addData("intake backward:", INTAKE_BACKWARD_VELOCITY);
                    forward = true;
                    intake.setPower(INTAKE_BACKWARD_VELOCITY);

                }

            }

//pusher servos
            if(gamepad2.x){
                telemetry.addLine("x is pressed");
                if (forward) {
                    forward = false;
                    pusher1.setPower(0.4);
                    pusher0.setPower(0.4);
                }else {
                    forward = true;
                    pusher1.setPower(-0.4);
                    pusher0.setPower(-0.4);
                    //telemetry.addLine("backwards");
                }
            }

            if(gamepad2.b){
                telemetry.addLine("b is pressed");

                pusher1.setPower(0.0);
                pusher0.setPower(0.0);
                forward = true;
            }

//elevator
            if(gamepad2.dpad_up){
                telemetry.addLine("dpad up is pressed");
                sqbeam.setPosition(0.2);
            }

            if(gamepad2.dpad_down){
                telemetry.addLine("dpad down is pressed");
                sqbeam.setPosition(0.8);
            }

//hogback launch state
            telemetry.addData("State", launchState);
            telemetry.addData("Hogback Velocity", hogback.getVelocity());



    }


    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayedDetectionTelemetry(AprilTagDetection detectedId) {
        if (detectedId == null) {
            return;
        }

        if (detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        } else {
            telemetry.addLine(String.format("\n====(ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }

    // A stops hogback


    void launch(boolean shotRequested) {
        switch (launchState) {

            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                hogback.setVelocity(HOGBACK_TARGET_VELOCITY);
                if (hogback.getVelocity() > HOGBACK_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;

            case LAUNCH:
                launchState = LaunchState.IDLE;
                break;

        }
    }



//    public AprilTagDetection getTagBySpecificId(int id) {
//        for (AprilTagDetection detection : detectedTags) {
//            if (detection.id == id) {
//                return detection;
//
//            }
//        }
//        return null;
//
//    }
//
//
//    public void stop() {
//        if (visionPortal != null) {
//            visionPortal.close();
//        }
//    }
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


