package org.firstinspires.ftc.teamcode.auto;
//test


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

//ignore this for now
@Autonomous(name="leaveBase")
public class leaveBase extends AutoHardware  {

    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;

    public leaveBase() {
    }


    @Override
    public void runOpMode() throws InterruptedException {
        initAll();

        //reset encoder
        setAutoDriveMotorMode();

        telemetry.update();
        waitForStart();


        encoderDrive(0.3,  -22,  -22, 15);

        sleep(1000);

        intakeIn();
        pusher0Set(0.8);
        pusher1Set(0.8);

        setHogbackTargetRpmFar();

        triggerShoot(3);



        // int forwardTicks = -2000; // strafe right
        // driveMotors(forwardTicks,-forwardTicks,-forwardTicks,forwardTicks, 0.5, false, robot.yaw0);





    }


}