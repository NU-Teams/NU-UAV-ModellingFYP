function LatLongAzimuth2 = VincentyFormula(LatLongAzimuth1, distance)

% unpack
phi1	= LatLongAzimuth1(1);
L1      = LatLongAzimuth1(2);
alpha1  = LatLongAzimuth1(3);
s       = distance;

% semi-axes of the Earth
a = 6378137.00;
b = 6356752.31;
f = 1-b/a;

%% Vincenty's Formula

U1      = atan((1-f)*tan(phi1));

sigma1  = atan2(tan(U1), cos(alpha1));

alpha   = asin(cos(U1)*sin(alpha1));

u2      = (cos(alpha)^2)*(a^2-b^2)/(b^2);

A       = 1 + (u2/16384)*(4096 + u2*(-768 + u2*(320 - 175*u2)));

B       = (u2/1024)*(256 + u2*(-128 + u2*(74-47*u2)));

sigma   = s/(b*A);

% iterate
d_sigma = 1;

while d_sigma > 1e-6
    sigma_m = sigma1 + sigma/2;

    d_sigma = B*sin(sigma)*(cos(2*sigma_m)...
            + (B/4)*(cos(sigma)*(-1+2*cos(2*sigma_m)^2)...
            - (B/6)*cos(2*sigma_m)*(-3 + 4*sin(sigma)^2)*(-3 + 4*cos(2*sigma_m)^2)));

    sigma   = s/(b*A) + d_sigma;
end

phi2    = atan2(sin(U1)*cos(sigma) + cos(U1)*sin(sigma)*cos(alpha1),...
                (1-f)*sqrt(sin(alpha)^2 + (sin(U1)*sin(sigma) - cos(U1)*cos(sigma)*cos(alpha1))^2));

lambda  = atan2(sin(sigma)*sin(alpha1),...
                cos(U1)*cos(sigma)-sin(U1)*sin(sigma)*cos(alpha1));

C       = (f/16)*(cos(alpha)^2)*(4+f*(4-3*cos(alpha)^2));

L       = lambda - (1-C)*f*sin(alpha)*(sigma + C*sin(sigma)*(cos(2*sigma_m) + C*cos(sigma)*(-1 + 2*cos(2*sigma_m)^2)));

L2      = L + L1;

alpha2  = atan2(sin(alpha), -sin(U1)*sin(sigma) + cos(U1)*cos(sigma)*cos(alpha1));

%% pack
latitude2 = phi2;
longitude2 = L2;
azimuth2 = alpha2;

LatLongAzimuth2 = [latitude2;
                   longitude2;
                   azimuth2];

end