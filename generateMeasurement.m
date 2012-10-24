function [cam, points] = generateMeasurement(p, X, Y, Z)
    % Given a point, generate measurements observing a sphere parameterized
    % by X Y Z

    import gtsam.*
    ps = Pose3(Rot3, Point3(p'));
    cam = SimpleCamera(ps);
    K = Cal3_S2(1000,1000,0,320,240);
    cam = cam.Lookat(Point3(p'), Point3(0,0,0), Point3(0,0,1), K);

    points = [];
    for i = 1:size(X,1)
        for j = 1:size(X,2)
            dist = sqrt((X(i,j) - p(1))^2 + (Y(i,j) - p(2))^2 + (Z(i,j) - p(3))^2);
            if(dist < 240000)
                points = [points; [X(i,j) Y(i,j) Z(i,j)]];
                cam.project(Point3(X(i,j), Y(i,j), Z(i,j)))
            end
        end
    end
    size(points)
end