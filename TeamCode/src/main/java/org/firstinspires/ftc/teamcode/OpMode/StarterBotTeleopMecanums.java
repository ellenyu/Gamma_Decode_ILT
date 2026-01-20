package org.firstinspires.ftc.teamcode.OpMode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import android.util.Size;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "StarterBotTeleopMecanums", group = "StarterBot")
public class StarterBotTeleopMecanums extends OpMode {

    final double FEED_TIME_SECONDS = 0.5;
    final double STOP_SPEED = 0.0;
    final double FULL_SPEED = -1;

    final double INTAKE_FORWARD_VELOCITY = 0.8;
    final double INTAKE_BACKWARD_VELOCITY = -0.6;

    boolean forward = true;

    final double BUMPER_FEED_TIME = 0.55;
    final double BUMPER_FEED_POWER = -0.3;

    private DcMotor leftFrontDrive, rightFrontDrive, leftBackDrive, rightBackDrive;
    private DcMotor intake;

    private Limelight3A limelight;

    private IMU imu;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;

    private List<AprilTagDetection> detectedTags = new ArrayList<>();

    private Telemetry telemetry;

    ElapsedTime feederTimer = new ElapsedTime();

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


        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.setMsTransmissionInterval(11);
        telemetry.addData("Status", "Initialized");

        imu = hardwareMap.get(IMU.class, "imu");

    // Adjust these two settings to match how your Hub is mounted!
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);

        imu.initialize(new IMU.Parameters(orientationOnRobot));


        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "limelight"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();




    }


    @Override
    public void loop() {

        mecanumDrive(
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

        // --- Limelight Logic (Replaces your while loop) ---
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            Pose3D botpose = result.getBotpose();
            telemetry.addData("tx", result.getTx());
            telemetry.addData("ty", result.getTy());
            telemetry.addData("Botpose", botpose.toString());
        }

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
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

        // Use getYaw() instead of firstAngle



        // Right trigger fires 3 shots
        if (gamepad2.right_bumper) {
            forward = true;
            intake.setPower(0);


        }

        if (gamepad2.left_bumper) {
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

    public void update() {
        detectedTags = aprilTagProcessor.getDetections();
    }


    public List<AprilTagDetection> getDetectedTags() {
        return detectedTags;
    }

    public void displayedDetectionTelemetry(AprilTagDetection detectedId){
        if (detectedId == null) {return;}

        if(detectedId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectedId.id, detectedId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f (inch)", detectedId.ftcPose.x, detectedId.ftcPose.y, detectedId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f (deg)", detectedId.ftcPose.pitch, detectedId.ftcPose.roll, detectedId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f (inch, deg, deg)", detectedId.ftcPose.range, detectedId.ftcPose.bearing, detectedId.ftcPose.elevation));
        }else {
            telemetry.addLine(String.format("\n====(ID %d) Unknown", detectedId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f (pixels)", detectedId.center.x, detectedId.center.y));
        }
    }


    public AprilTagDetection getTagBySpecificId(int id) {
        for (AprilTagDetection detection : detectedTags) {
            if (detection.id == id) {
                return detection;

            }
        }
        return null;

    }

    public void stop() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
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


