function R = ComputeAngleAxis(theta,v)
    %Implement here the Rodrigues formula
    I = eye(3);
    vx = v(1);
    vy = v(2);
    vz = v(3);
    hx = [0 -vz vy;
        vz 0 -vx;
        -vy vx 0];
    R = I + (hx*sin(theta)) + ((hx^2)*(1-cos(theta)));
end
