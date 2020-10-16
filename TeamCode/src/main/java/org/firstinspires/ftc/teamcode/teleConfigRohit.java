package org.firstinspires.ftc.teamcode;

public class teleConfigRohit implements teleOpInterface {
    HardwareMapV2 robot;
    Drivetrain drivetrain;
    static boolean wait = false;
    static boolean up = false;
    enum mode{
        INCREMENT, POSITION
    }
    static mode tiltmode = mode.POSITION;
    static double inttakePower = 1.0;
    static double outtakePower = 1.0;
    static double conveyorPower = 1.0;

    public void a() {
        conveyorPower-=0.1;
    }

    public void b() {

    }

    public void x() {
        conveyorPower+=0.1;
    }

    public void y() {

    }

    public void dd() {

    }

    public void dp() {

    }

    public void dl() {

    }

    public void dr() {

    }

    public void rb() {
        drivetrain.outtakeAll(conveyorPower, outtakePower);
    }

    public void rt() {
        drivetrain.outtakeAll(-conveyorPower, -outtakePower);
    }

    public void lb() {
        robot.intake.setPower(inttakePower);
    }

    public void lt() {
        robot.intake.setPower(-inttakePower);
    }

    public void rjoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y==0 && wait) {
            switch (tiltmode) {
                case POSITION:
                        drivetrain.tiltpos((up) ? Drivetrain.tiltDirect.UP : Drivetrain.tiltDirect.DOWN);
                    break;
                case INCREMENT:
                    drivetrain.tilt(robot.leftTilt.getPosition() + ((up) ? 0.1 : -0.1));
                    break;
            }
            wait = false;
        }
    }

    public void ljoy(float x, float y) {
        if (Math.abs(y)>0.75){ wait = true; up = y > 0; }
        if (y==0 && wait) {
            inttakePower = (up && inttakePower!=1.0) ? inttakePower*2 : (!up && inttakePower!=0.125) ? inttakePower/2 : 1.0;
        }
        wait = false;
    }

    public void rjoyb() {
        tiltmode = (tiltmode==mode.POSITION) ? mode.INCREMENT : mode.POSITION;
    }

    public void ljoyb() {
        inttakePower = 1.0;
    }

    public void custom1() {

    }
}
