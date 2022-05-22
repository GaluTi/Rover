#ifndef DRAWWIDGET_H
#define DRAWWIDGET_H

#include <QWidget>

const float R = 2;
const float S = 6;
const float S_T_W_D = 6;
const double eps = 1;
const float suspension_angle = M_PI/3.5;
const float delta_suspension = R/4;

struct Vector2D {

    double x;
    double y;

    Vector2D() { x = 0.0; y = 0.0; }
    Vector2D(double _x, double _y) { x = _x, y = _y; }
    Vector2D(Vector2D begin, Vector2D end) { x = end.x - begin.x, y = end.y - begin.y; }

    double length() {
        return sqrt(x*x + y*y);
    }
    void normalize() {
        double l = length();
        if(l > 0) {
            x /= l;
            y /= l;
        }
    }

    Vector2D operator+(const Vector2D& v) const {
        return Vector2D(x + v.x, y + v.y);
    }
    Vector2D operator-(const Vector2D& v) const {
        return Vector2D(x - v.x, y - v.y);
    }
    double operator*(const Vector2D& v) const {
        return x * v.x + y * v.y;
    }
    Vector2D operator*(const double& f) const {
        return Vector2D(x * f, y * f);
    }
    bool operator==(const Vector2D& v) const {
        return (fabs(x - v.x) < eps && fabs(y - v.y) < eps);
    }
    bool operator!=(const Vector2D& v) const {
        return (fabs(x - v.x) > eps || fabs(y - v.y) > eps);
    }
};


class DrawWidget : public QWidget
{
    Q_OBJECT
public:
    explicit DrawWidget(QWidget *parent = nullptr);

    std::vector<Vector2D> surface;
    std::vector<std::pair<Vector2D, Vector2D>> wheel_center_line;
    Vector2D start_point;
    Vector2D first_wheel_center;
    Vector2D second_wheel_center;
    Vector2D third_wheel_center;
    Vector2D fourth_wheel_center;
    bool first_wheel;
    bool second_wheel;
    bool third_wheel;
    bool fourth_wheel;
    Vector2D first_suspension_center;
    Vector2D second_suspension_center;
    std::vector<std::pair<Vector2D, Vector2D>> suspension;
    std::vector<Vector2D> rover_body;
    bool is_suspension;
    bool is_rover_body;

    DrawWidget();
    void paintEvent(QPaintEvent *event);
};

#endif // DRAWWIDGET_H
