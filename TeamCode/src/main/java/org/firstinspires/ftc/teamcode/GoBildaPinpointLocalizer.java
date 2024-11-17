package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Twist2dDual;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.Vector2dDual;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * This class implements localization for the gobilda pinpoint odometry computer.
 */
public class GoBildaPinpointLocalizer implements Localizer
{
    // Millimeter forward/back offset of the lateral measuring pod (relative to center point)
    private static final double X_OFFSET_MM = 70.0;
    // Millimeter left/right offset of the forward measuring pod (relative to center point)
    private static final double Y_OFFSET_MM = -165.0;

    // Units for pinpoint odometry computer are mm and degrees
    private GoBildaPinpointDriver odo;
    private Pose2D last_pos_mm;
    private Pose2D last_vel_mm_s;
    private double last_heading_radians;
    private boolean initialized = false;

    public GoBildaPinpointLocalizer(HardwareMap hardwareMap){
        // Get the driver from the hardware map and setup
        odo = hardwareMap.get(GoBildaPinpointDriver.class,"odo");
        odo.setOffsets(X_OFFSET_MM, Y_OFFSET_MM);
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.REVERSED);
        odo.resetPosAndIMU();
    }

    @Override
    public Twist2dDual<Time> update() {

        // Get odometry values
        odo.update();
        Pose2D pos_mm = odo.getPosition();
        Pose2D vel_mm_s = odo.getVelocity();
        double heading_radians = odo.getHeading();
        double heading_velocity_radians_s = odo.getHeadingVelocity();

        // Build twist
        Twist2dDual<Time> twist;
        if (!initialized) {
            initialized = true;
            twist = new Twist2dDual<>(
                Vector2dDual.constant(new Vector2d(0.0, 0.0), 2),
                DualNum.constant(0.0, 2)
            );
        }
        else {
            twist = new Twist2dDual<>(
                new Vector2dDual<>(
                    new DualNum<Time>(new double[]{
                        pos_mm.getX(DistanceUnit.INCH) - last_pos_mm.getX(DistanceUnit.INCH),
                        vel_mm_s.getX(DistanceUnit.INCH)}),
                    new DualNum<Time>(new double[]{
                        pos_mm.getY(DistanceUnit.INCH) - last_pos_mm.getY(DistanceUnit.INCH),
                        vel_mm_s.getY(DistanceUnit.INCH)})),
                new DualNum<>(new double[]{
                    heading_radians - last_heading_radians,
                    heading_velocity_radians_s}));
        }

        // Remember what we just saw
        last_pos_mm = pos_mm;
        last_vel_mm_s = vel_mm_s;
        last_heading_radians  = heading_radians;

        // Done
        return twist;
    }

    /**
     * Resets the position and inertial motion unit
     */
    public void reset(){
        odo.resetPosAndIMU();
    }

    public double getx_in()
    {
        Pose2D pos_mm = odo.getPosition();
        return pos_mm.getX(DistanceUnit.INCH);
    }

    public double gety_in()
    {
        Pose2D pos_mm = odo.getPosition();
        return pos_mm.getY(DistanceUnit.INCH);
    }

    public double getheading_radians()
    {
        Pose2D pos_mm = odo.getPosition();
        return pos_mm.getHeading(AngleUnit.RADIANS);
    }

    public Pose2d get_pose_estimate()
    {
        Pose2D pos_mm = odo.getPosition();
        return new Pose2d(
                pos_mm.getX(DistanceUnit.INCH),
                pos_mm.getY(DistanceUnit.INCH),
                pos_mm.getHeading(AngleUnit.RADIANS));
    }

    public PoseVelocity2d get_velocity_estimate()
    {
        Pose2D vel_mm_s = odo.getVelocity();
        return new PoseVelocity2d(
                new Vector2d(
                vel_mm_s.getX(DistanceUnit.INCH),
                vel_mm_s.getY(DistanceUnit.INCH)),
                vel_mm_s.getHeading(AngleUnit.RADIANS));
    }
}
