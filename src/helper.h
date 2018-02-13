using namespace std;

 

class checkCar {
public:
  double _vx, _vy;
  double _s, _d; 
  checkCar(double vx, double vy, double s, double d);
};

class egoCar {
public:
  double _x, _y, _s, _d, _yaw, _speed;
  egoCar(double x, double y, double s, double d, double yaw,
  double speed);
};

double distance(double x1, double y1, double x2, double y2);

int ClosestWaypoint(double x, double y, const vector<double> &maps_x, const vector<double> &maps_y);


int NextWaypoint(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);


// Transform from Cartesian x,y coordinates to Frenet s,d coordinates
vector<double> getFrenet(double x, double y, double theta, const vector<double> &maps_x, const vector<double> &maps_y);


// Transform from Frenet s,d coordinates to Cartesian x,y
vector<double> getXY(double s, double d, const vector<double> &maps_s, const vector<double> &maps_x, const vector<double> &maps_y);

