#ifndef DRAWWIDGET_H
#define DRAWWIDGET_H

#include <QWidget>
#include <QPainter>
#include <QMouseEvent>
#include <QPen>
#include <math.h>

const float R = 2; //Радиус колеса
const float S = 6; //Расстояние между колёсами
const float suspension_distance = 14; // Расстояние между центрами подвесок
const double eps = 0.01; //Погрешность
const float suspension_angle = M_PI/3.5; //Угол при основании подвески
const float suspension_height = S * tan(suspension_angle) / 2; // Высота в треугольнике подвески
const float suspension_edge = S / (2 * cos(suspension_angle)); //Боковая сторона треугольника подвески
const float delta_suspension = R/4; //Расстояние, на которое отстоит верхняя и нижняя части подвески, от линии, соединяющей цетры колеса и подвески
const int slider_size = 5000; //Количество шагов слайдера

//Двумерный вектор
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
    Vector2D operator/(const double& f) const {
        return Vector2D(x / f, y / f);
    }
    bool operator==(const Vector2D& v) const {
        return (fabs(x - v.x) < eps && fabs(y - v.y) < eps);
    }
    bool operator!=(const Vector2D& v) const {
        return (fabs(x - v.x) > eps || fabs(y - v.y) > eps);
    }
};

//Данный виджет занимается отрисовкой всего
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
    Vector2D first_suspension_center;
    Vector2D second_suspension_center;
    std::vector<std::pair<Vector2D, Vector2D>> suspension; //Подвеска представляет из себя вектор пар точек, где каждая пара точек - ребро подвески. Всего в одной подвеске 4 ребра
    std::vector<Vector2D> rover_body;
    bool surface_calculated;
    bool wheel_center_line_calculated;
    bool first_wheel_calculated;
    bool second_wheel_calculated;
    bool third_wheel_calculated;
    bool fourth_wheel_calculated;
    bool suspension_calculated;
    bool rover_body_calculated;
    bool surface_check;
    bool wheel_center_line_check;
    bool rover_check;

    DrawWidget();
    void paintEvent(QPaintEvent *event);
};

#endif // DRAWWIDGET_H
