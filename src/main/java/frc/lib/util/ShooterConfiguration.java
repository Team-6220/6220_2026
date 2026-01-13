package frc.lib.util;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;

public class ShooterConfiguration {
    private Pair<Double, Double> velocities = Pair.of(0.0, 0.0);
    private double armAngle;
    private double headingOffset;
    public static HashMap<Integer, Double> radiusValues = new HashMap<Integer, Double>();
    private static HashMap<Pair<Double, Integer>, ShooterConfiguration> shooterConfigurations = new HashMap<Pair<Double, Integer>, ShooterConfiguration>();
    private static ArrayList<ArrayList<Pair<Double, Integer>>> keys = new ArrayList<ArrayList<Pair<Double, Integer>>>();
    private static double GRID_WIDTH = 8 * Math.PI / 9;

    private ShooterConfiguration(Pair<Double, Double> velocities, double armAngle, double headingOffset){
        this.velocities = velocities;
        this.armAngle = armAngle;
        this.headingOffset = headingOffset;
    }

    public Pair<Double, Double> getVelocities(){
        return velocities;
    }

    public double getArmAngle(){
        return armAngle;
    }

    public double getHeadingOffset(){
        return headingOffset;
    }

    public void setVelocities(Pair<Double, Double> velocities){
        this.velocities = velocities;
    }

    public static void setupRadiusValues(){
        radiusValues.put(1, 1.330325);
        radiusValues.put(2, 2.0);
        radiusValues.put(3, 2.5);
        radiusValues.put(4, 3.0);
        radiusValues.put(5, 3.5);
    }

    public static void setupKeys(){
        keys.add(null);
        for(int i = 1; i < 6; i++){
            ArrayList<Pair<Double, Integer>> inner = new ArrayList<Pair<Double, Integer>>();
            for(int j = 0; j <= (i + 1) * 2; j++){
                inner.add(Pair.of(radiusValues.get(i), j));
            }
            keys.add(inner);
        }
    }
    public static void setupConfigurations(){
        setupKeys();
        //FIXME - Add actual values - refer to desmos graph to see corresponding angles
        shooterConfigurations.put(keys.get(1).get(0), new ShooterConfiguration(Pair.of(3700d,3900d), 68d, 8d));
        shooterConfigurations.put(keys.get(1).get(1), new ShooterConfiguration(Pair.of(3700d,3900d), 68d, 0d));
        shooterConfigurations.put(keys.get(1).get(2), new ShooterConfiguration(Pair.of(3700d,3900d), 68d, 0d));
        shooterConfigurations.put(keys.get(1).get(3), new ShooterConfiguration(Pair.of(3700d,3900d), 68d, 0d));
        shooterConfigurations.put(keys.get(1).get(4), new ShooterConfiguration(Pair.of(3700d,3900d), 69d, 5d));
        
        shooterConfigurations.put(keys.get(2).get(0), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 5d)); //fails
        shooterConfigurations.put(keys.get(2).get(1), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 0d));
        shooterConfigurations.put(keys.get(2).get(2), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 0d));
        shooterConfigurations.put(keys.get(2).get(3), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 5d));
        shooterConfigurations.put(keys.get(2).get(4), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 5d));
        shooterConfigurations.put(keys.get(2).get(5), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 5d));
        shooterConfigurations.put(keys.get(2).get(6), new ShooterConfiguration(Pair.of(3700d,3900d), 59d, 8d));//fails
        
        shooterConfigurations.put(keys.get(3).get(0), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 3d)); //fails
        shooterConfigurations.put(keys.get(3).get(1), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));
        shooterConfigurations.put(keys.get(3).get(2), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));
        shooterConfigurations.put(keys.get(3).get(3), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));
        shooterConfigurations.put(keys.get(3).get(4), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));
        shooterConfigurations.put(keys.get(3).get(5), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 8d));
        shooterConfigurations.put(keys.get(3).get(6), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 5d));
        shooterConfigurations.put(keys.get(3).get(7), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));
        shooterConfigurations.put(keys.get(3).get(8), new ShooterConfiguration(Pair.of(3700d,3900d), 52d, 4d));//fails

        shooterConfigurations.put(keys.get(4).get(0), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 4d)); //fails   
        shooterConfigurations.put(keys.get(4).get(1), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 4d));

        shooterConfigurations.put(keys.get(4).get(2), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 2d));
        shooterConfigurations.put(keys.get(4).get(3), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 2d));
        shooterConfigurations.put(keys.get(4).get(4), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 0d));
        shooterConfigurations.put(keys.get(4).get(5), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 5d));
        shooterConfigurations.put(keys.get(4).get(6), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 8d));
        shooterConfigurations.put(keys.get(4).get(7), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 10d));
        shooterConfigurations.put(keys.get(4).get(8), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 10d));
        shooterConfigurations.put(keys.get(4).get(9), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 10d));
        shooterConfigurations.put(keys.get(4).get(10), new ShooterConfiguration(Pair.of(3700d,3900d), 50d, 10d));//fails

        shooterConfigurations.put(keys.get(5).get(0), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5d, 5d)); //fails or couldnt test
        shooterConfigurations.put(keys.get(5).get(1), new ShooterConfiguration(Pair.of(3800d,4300d), 45.6d, 5d)); //fails or couldnt test
        shooterConfigurations.put(keys.get(5).get(2), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 3d)); //couldnt test
        shooterConfigurations.put(keys.get(5).get(3), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 0d));
        shooterConfigurations.put(keys.get(5).get(4), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5d, 0d));
        shooterConfigurations.put(keys.get(5).get(5), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5d, 0d));
        shooterConfigurations.put(keys.get(5).get(6), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 5d));
        shooterConfigurations.put(keys.get(5).get(7), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d));
        shooterConfigurations.put(keys.get(5).get(8), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d));
        shooterConfigurations.put(keys.get(5).get(9), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d)); //
        shooterConfigurations.put(keys.get(5).get(10), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d)); //
        shooterConfigurations.put(keys.get(5).get(11), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d));//
        shooterConfigurations.put(keys.get(5).get(12), new ShooterConfiguration(Pair.of(3800d,4300d), 45.5, 7d)); //
    }

    public static Pair<Double, Double> polarToCartesian(double r, int num){
        double angle = ((GRID_WIDTH) * (num / ((r + 1) * 2)) - (GRID_WIDTH / 2));
        Pair<Double, Double> pos = Pair.of(r * Math.cos(angle), r * Math.sin(angle));
        
        if(Constants.isRed.equals("red")) {
            pos = Pair.of(VisionConstants.SPEAKER_POSE2D_RED.getX() - pos.getFirst(), VisionConstants.SPEAKER_POSE2D_RED.getY() - pos.getSecond());
        } else {
            pos = Pair.of(VisionConstants.SPEAKER_POSE2D_BLUE.getX() + pos.getFirst(), VisionConstants.SPEAKER_POSE2D_BLUE.getY() + pos.getSecond());
        }
        return pos;
    }

    private static Pair<Double, Double> cartesianToPolar(Pose2d pose){
        Pose2d speakerPos = Constants.isRed.equals("red") ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;
        double yval = Constants.isRed.equals("red") ? VisionConstants.SPEAKER_POSE2D_RED.getY() - pose.getY() : pose.getY() - VisionConstants.SPEAKER_POSE2D_BLUE.getY();
        double angle = Math.atan2(yval, Math.abs(speakerPos.getX() - pose.getX()));
        double r = Math.hypot(Math.abs(pose.getX() - speakerPos.getX()), Math.abs(pose.getY() - speakerPos.getY()));
        return Pair.of(r, angle);
    }

    private static double columnToAngle(int col, int numcol){
        return ((GRID_WIDTH) * ((double)col / (double)(numcol))) - (GRID_WIDTH / 2);
    }

    private static List<Pair<Integer, Integer>> getNearestShooterConfigurations(Pose2d robotPose) throws IllegalPositionException{
        ArrayList<Pair<Integer, Integer>> nearestConfigurations = new ArrayList<Pair<Integer, Integer>>();
        Pair<Double, Double> polar = cartesianToPolar(robotPose);
        // System.out.println("Polar Relative to Speaker, R: " + polar.getFirst() + "Theta: " + polar.getSecond());
        int innerRow = (int) Math.floor(polar.getFirst());
        int outerRow = (int) Math.ceil(polar.getFirst());
        if(polar.getSecond() <= -(GRID_WIDTH / 2) || polar.getSecond() >= (GRID_WIDTH / 2)){
            throw new IllegalPositionException("Robot is too far to the side");
        }
        else if(polar.getFirst() < 1 || polar.getFirst() > 3.5){
            throw new IllegalPositionException("Robot is too far away");
        }

        // System.out.println(innerRow + " " + outerRow);
        for(int i = 0; i < (innerRow + 1) * 2; i++){
            if(polar.getSecond() >= columnToAngle(i, (innerRow + 1) * 2) && polar.getSecond() <= columnToAngle(i+1, (innerRow + 1) * 2)){
                nearestConfigurations.add(Pair.of(innerRow, i));
                nearestConfigurations.add(Pair.of(innerRow, i + 1));
                
                //System.out.println("It is adding something");
                break;
            }
            //System.out.println("Inner For Loop columnToAngle 1: " + columnToAngle(i, (innerRow + 1) * 2) + " 2:" + columnToAngle(i+1, (innerRow + 1) * 2));
        }
        for(int i = 0; i < (outerRow + 1) * 2; i++){
            if(polar.getSecond() >= columnToAngle(i, (outerRow + 1) * 2) && polar.getSecond() <= columnToAngle(i+1, (outerRow + 1) * 2)){
                nearestConfigurations.add(Pair.of(outerRow, i));
                nearestConfigurations.add(Pair.of(outerRow, i + 1));
                
                break;
            }
        }
        
        //System.out.println("Length before culling: " + nearestConfigurations.size());
        Pair<Integer, Integer> furthest = null;
        double max = 0;
        for (Pair<Integer, Integer> point : nearestConfigurations){
            Pair<Double, Double> pointCartesian = polarToCartesian(radiusValues.get(point.getFirst()), point.getSecond());
            if(Math.hypot(pointCartesian.getFirst() - robotPose.getX(), pointCartesian.getSecond() - robotPose.getY()) > max){
                max = Math.hypot(pointCartesian.getFirst() - robotPose.getX(), pointCartesian.getSecond() - robotPose.getY());
                furthest = point;
            }
        }
        
        nearestConfigurations.remove(furthest);
        //System.out.println("Got Nearest Configs with length: " + nearestConfigurations.size());
        return nearestConfigurations;
    }

    private static double[] get_equation_plane(double x1, double y1, double z1, double x2, double y2, double z2, double x3, double y3, double z3){
        double a1 = x2 - x1;
        double b1 = y2 - y1;
        double c1 = z2 - z1;
        double a2 = x3 - x1;
        double b2 = y3 - y1;
        double c2 = z3 - z1;
        double a = b1 * c2 - b2 * c1;
        double b = a2 * c1 - a1 * c2;
        double c = a1 * b2 - b1 * a2;
        double d = (- a * x1 - b * y1 - c * z1);
        return new double[]{a, b, c, d};
    }

    private static double solve_equation_plane(double[] coeffs, double x, double y){
        return (-coeffs[3] - coeffs[0] * x - coeffs[1] * y) / coeffs[2];
    }

    public static ShooterConfiguration getShooterConfiguration(Pose2d robotPose) throws IllegalPositionException{
        List<Pair<Integer, Integer>> configs = getNearestShooterConfigurations(robotPose);
        //Pose2d speakerPos = Constants.isRed.equals("red") ? VisionConstants.SPEAKER_POSE2D_RED : VisionConstants.SPEAKER_POSE2D_BLUE;
        double yval = robotPose.getY();//Constants.isRed.equals("red") ? VisionConstants.SPEAKER_POSE2D_RED.getY() - robotPose.getY() : robotPose.getY() - VisionConstants.SPEAKER_POSE2D_BLUE.getY();
        double xval = robotPose.getX();//Math.abs(robotPose.getX() - speakerPos.getX());
        Pair<Double, Double> a = polarToCartesian(configs.get(0).getFirst(), configs.get(0).getSecond());
        Pair<Double, Double> b = polarToCartesian(configs.get(1).getFirst(), configs.get(1).getSecond());
        Pair<Double, Double> c = polarToCartesian(configs.get(2).getFirst(), configs.get(2).getSecond());
        //System.out.println(shooterConfigurations.get(configs.get(0)).getVelocities().getFirst());
        int[][] points = {{configs.get(0).getFirst(), configs.get(0).getSecond()}, {configs.get(1).getFirst(), configs.get(1).getSecond()}, {configs.get(2).getFirst(), configs.get(2).getSecond()}};
        double[] v1coeffs = get_equation_plane(
            a.getFirst(), a.getSecond(), shooterConfigurations.get(keys.get(points[0][0]).get(points[0][1])).getVelocities().getFirst(),
            b.getFirst(), b.getSecond(), shooterConfigurations.get(keys.get(points[1][0]).get(points[1][1])).getVelocities().getFirst(),
            c.getFirst(), c.getSecond(), shooterConfigurations.get(keys.get(points[2][0]).get(points[2][1])).getVelocities().getFirst()
        );

        double[] v2coeffs = get_equation_plane(
            a.getFirst(), a.getSecond(), shooterConfigurations.get(keys.get(points[0][0]).get(points[0][1])).getVelocities().getSecond(),
            b.getFirst(), b.getSecond(), shooterConfigurations.get(keys.get(points[1][0]).get(points[1][1])).getVelocities().getSecond(),
            c.getFirst(), c.getSecond(), shooterConfigurations.get(keys.get(points[2][0]).get(points[2][1])).getVelocities().getSecond()
        );

        double[] armcoeffs = get_equation_plane(
            a.getFirst(), a.getSecond(), shooterConfigurations.get(keys.get(points[0][0]).get(points[0][1])).getArmAngle(),
            b.getFirst(), b.getSecond(), shooterConfigurations.get(keys.get(points[1][0]).get(points[1][1])).getArmAngle(),
            c.getFirst(), c.getSecond(), shooterConfigurations.get(keys.get(points[2][0]).get(points[2][1])).getArmAngle()
        );
        double[] headingcoeffs = get_equation_plane(
            a.getFirst(), a.getSecond(), shooterConfigurations.get(keys.get(points[0][0]).get(points[0][1])).getHeadingOffset(),
            b.getFirst(), b.getSecond(), shooterConfigurations.get(keys.get(points[1][0]).get(points[1][1])).getHeadingOffset(),
            c.getFirst(), c.getSecond(), shooterConfigurations.get(keys.get(points[2][0]).get(points[2][1])).getHeadingOffset()
        );
        Pair<Double, Double> vels = Pair.of(solve_equation_plane(v1coeffs, xval, yval), solve_equation_plane(v2coeffs, xval, yval));
        double arm = solve_equation_plane(armcoeffs, xval, yval);
        double heading = solve_equation_plane(headingcoeffs, xval, yval);
        //System.out.println("New Shooter Config Created at: " + xval + ", " + yval + "\nArm Angle: " + arm + "\nVelocity: " + vels.getFirst() + "\nHeading Offset: " + heading);
        return new ShooterConfiguration(vels, arm, heading);
    }
}
