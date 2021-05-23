function [FPA, RNG, D_OBS, ALT, warn] = fcn(OBS, CURRENT, TARGET)

d2r = pi / 180;   %degree to radians
R = 6371e3;

% OBSTACLE LOCATION
LAT_OBS = OBS(1);
LON_OBS = OBS(2);
ELEV_OBS = OBS(3);

% OBSTACLE THRESHOLD ZONE
thres = 3000;

% TARGET LOCATION
LAT_TARGET = TARGET(1);
LON_TARGET = TARGET(2);
ELEV_TARGET = TARGET(3);

% CURRENT LOCATION
ELEV_CUR = CURRENT(1);
LAT_CUR = CURRENT(2);
LON_CUR = CURRENT(3);

% DISTANCE TO TARGET
l1 = LAT_CUR*d2r;
u1 = LON_CUR*d2r;
l2 = LAT_TARGET*d2r;
u2 = LON_TARGET*d2r;
dh = abs(ELEV_TARGET - ELEV_CUR);

d1 = l2 - l1;
du = u2 - u1;

%harveersine formula (distance across the floor not the range)
a = sin(d1/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2; 
c = 2*atan2(sqrt(a),sqrt(1-a));
d = R*c; % horizontal distance (in m);

% DISTANCE FROM THE OBSTACLE
l3 = LAT_OBS*d2r;
u3 = LON_OBS*d2r;

d1 = l3 - l1;
du = u3 - u1;

a = sin(d1/2)^2 + cos(l1)*cos(l2)*sin(du/2)^2; 
c = 2*atan2(sqrt(a),sqrt(1-a));
d_obs = R*c; 

% current range (from target) - range > distance
range = sqrt(d^2 + dh^2);

%calculate commanded flight path setpoint based on d_obs, range
if abs(d_obs) >= thres
    w = 0;
    FPA_CMD = atan(dh/d);
else
    w = 1;
    FPA_CMD = atan(-abs((ELEV_CUR*1.01 - ELEV_OBS)/d_obs));
end
FPA = FPA_CMD;
RNG = range;
D_OBS = d_obs;
ALT = ELEV_CUR;
warn = w;


