package robot.subsystems.drivetrain.pure_pursuit;

public class Main {
    public static void main(String[] args){
        Point a = new Point(1,0);
        Point b = new Point(2,2);
        Point c = new Point(1.6,1);
        System.out.println(VectorPursuit.velocityByDistance(0,0.5,0,1));
        System.out.println(VectorPursuit.velocityByDistance(0,0.5,0,1));

        System.out.println(VectorPursuit.velocityByDistance(0,0.5,0,1));
        System.out.println(VectorPursuit.velocityByDistance(0,-0.5,1,0));

    }
}
