function [cam, points] = generateMeasurement(p, X, Y, Z, K)
    % Given a point, generate measurements observing a sphere parameterized
    % by X Y Z

    import gtsam.*
    ps = Pose3(Rot3, Point3(p'));
    cam = SimpleCamera(ps);
 
    cam = cam.Lookat(Point3(p'), Point3(0,0,0), Point3(0,0,1), K);

    points = [];
    id = 0;
    % Grab the closest 1/2 of points
    for i = 1:size(X,1)
        for j = 1:size(X,2)
            dist = sqrt((X(i,j) - p(1))^2 + (Y(i,j) - p(2))^2 + (Z(i,j) - p(3))^2);
            points = [points; [X(i,j) Y(i,j) Z(i,j) dist id]];
            id = id + 1;
%                 cam.project(Point3(X(i,j), Y(i,j), Z(i,j)));
        end
    end
    points = sortrows(points, 4);
    % Floor to suppress warning about interger only ops with :
    points = points(1:floor(size(points,1)/10),:);
%     size(points);
end