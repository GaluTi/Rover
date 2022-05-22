#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <cmath>

/*const double R = 2;
const double S = 6;
const double eps = 1e-7;

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

double CrossProduct(Vector2D a, Vector2D b) {
    return a.x * b.y - a.y * b.x;
}

class LunarRover {
    public:

    private:
    float _mass;
    float _suspensionWheelDist;
    float _wheelRadius;
    std::vector<Vector2D> _wheels[4];

};

//Деление строки по пробелам
std::vector<std::string> Split(std::string str) {
    std::vector<std::string> result;
    std::string current_string = "";

    for(unsigned int i = 0 ; i < str.length(); ++i) {
        if(str[i] != ' ') {
            current_string.push_back(str[i]);
        }
        else {
            result.push_back(current_string);
            current_string = "";
        }
    }
    result.push_back(current_string);

    return result;
}

//Параллельный перенос отрезка на h вверх
std::pair<Vector2D, Vector2D> ParallelTransfer(Vector2D begin_point, Vector2D end_point, double height) {
    std::pair<Vector2D, Vector2D> result;
    Vector2D line_direction(begin_point, end_point);
    result.first.x = begin_point.x - height * line_direction.y / line_direction.length();
    result.first.y = begin_point.y + height * line_direction.x / line_direction.length();
    result.second.x = end_point.x - height * line_direction.y / line_direction.length();
    result.second.y = end_point.y + height * line_direction.x / line_direction.length();
    return result;
}

//Поиск точки пересечения двух прямых y(x) = k1x + b1 и y(x) = k2x + b2
Vector2D CrossPoint(Vector2D begin_point_1, Vector2D end_point_1, Vector2D begin_point_2, Vector2D end_point_2) {
    double k1 = (end_point_1.y - begin_point_1.y) / (end_point_1.x - begin_point_1.x);
    double b1 = (begin_point_1.y * end_point_1.x - end_point_1.y * begin_point_1.x) / (end_point_1.x - begin_point_1.x);
    double k2 = (end_point_2.y - begin_point_2.y) / (end_point_2.x - begin_point_2.x);
    double b2 = (begin_point_2.y * end_point_2.x - end_point_2.y * begin_point_2.x) / (end_point_2.x - begin_point_2.x);
    double x = (b2 - b1) / (k1 - k2);
    double y = (k1 * b2 - k2 * b1) / (k1 - k2);
    Vector2D res(x, y);
    return res;
}

//Решение квадратного уравнения Ax^2 + Bx + C = 0
int SquareEquation(double A, double B, double C, std::vector<double> &roots) {
    roots.clear();
    double D = B * B - 4 * A * C;
    if(D < -eps) {
        roots.push_back(-1);
        return -1;
    } else if(D > eps) {
        roots.push_back(((-B) + sqrt(D)) / (2 * A));
        roots.push_back(((-B) - sqrt(D)) / (2 * A));
        return 1;
    } else {
        roots.push_back((-B) / (2 * A));
        return 0;
    }
}

//Проверка пренадлежности точки прямой y(x) = kx + b
bool IsOnLine(Vector2D line_begin_point, Vector2D line_end_point, Vector2D point) {
    double k = (line_end_point.y - line_begin_point.y) / (line_end_point.x - line_begin_point.x);
    double b = (line_begin_point.y * line_end_point.x - line_end_point.y * line_begin_point.x) / (line_end_point.x - line_begin_point.x);
    if(fabs(point.y - (k * point.x + b)) < eps && line_begin_point.x - point.x < eps && point.x - line_end_point.x < eps) {
        return true;
    } else {
        return false;
    }
}

//Проверка корректности заданной на поверхности точки
bool IsCorrectSurfacePoint(const std::vector<Vector2D> &_surface, const std::vector<std::pair<Vector2D, Vector2D>> &_wheel_center_line, Vector2D point, Vector2D &wheel_center_point, int &wheel_center_line_num) {
    int check = 0;
    Vector2D line_begin;
    Vector2D line_end;
    for(unsigned int i = 0; i < _surface.size() - 1; ++i) {
        line_begin = _surface[i];
        line_end = _surface[i + 1];
        if(IsOnLine(line_begin, line_end, point)) {
            check = 1;
            wheel_center_point = ParallelTransfer(line_begin, point, R).second;
            break;
        }
    }
    if(check == 0) {
        return false;
    }

    for(unsigned int i = 0; i < _wheel_center_line.size() - 1; ++i) {
        line_begin = _wheel_center_line[i].first;
        line_end = _wheel_center_line[i + 1].first;
        if(_wheel_center_line[i].second != _wheel_center_line[i + 1].second) {
            if(IsOnLine(line_begin, line_end, wheel_center_point)) {
                check = 2;
                wheel_center_line_num = i + 1;
                break;
            }
        }
    }
    if(check != 2) {
        return false;
    }

    return true;
}

//Получение координат второго колеса по координатам первого
Vector2D SecondWheelCenter(const std::vector<Vector2D> &_surface, const std::vector<std::pair<Vector2D, Vector2D>> &_wheel_center_line, Vector2D begin_point) {
    Vector2D first_wheel_center;
    Vector2D second_wheel_center;
    Vector2D line_begin;
    Vector2D line_end;
    int line_num = -1;

    if(IsCorrectSurfacePoint(_surface, _wheel_center_line, begin_point, first_wheel_center, line_num)) {
        //Поиск координат центра второго колеса
        for(unsigned int i = line_num; i > 0; --i) {
            line_begin = _wheel_center_line[i - 1].first;
            line_end = _wheel_center_line[i].first;
            std::vector<double> roots;
            if(_wheel_center_line[i].second != _wheel_center_line[i - 1].second) {
                double k = (line_end.y - line_begin.y) / (line_end.x - line_begin.x);
                double b = (line_begin.y * line_end.x - line_end.y * line_begin.x) / (line_end.x - line_begin.x);
                double A = k * k + 1;
                double B = 2 * k * b - 2 * first_wheel_center.x - 2 * first_wheel_center.y * k;
                double C = first_wheel_center.x * first_wheel_center.x + first_wheel_center.y * first_wheel_center.y - 2 * first_wheel_center.y * b + b * b - S * S;
                int res = SquareEquation(A, B, C, roots);
                if(res == 0) {
                    double x2c = roots[0];
                    double y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c - first_wheel_center.x < eps) {
                        second_wheel_center.x = x2c;
                        second_wheel_center.y = y2c;
                        break;
                    }
                } else if(res == 1) {
                    double x2c = roots[0];
                    double y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c - first_wheel_center.x < eps) {
                        second_wheel_center.x = x2c;
                        second_wheel_center.y = y2c;
                        break;
                    } else {
                        x2c = roots[1];
                        y2c = k * x2c + b;
                        if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c - first_wheel_center.x < eps) {
                            second_wheel_center.x = x2c;
                            second_wheel_center.y = y2c;
                            break;
                        }
                    }
                }
            } else {
                Vector2D origin_point = _wheel_center_line[i].second;
                double k = - (origin_point.x - first_wheel_center.x) / (origin_point.y - first_wheel_center.y);
                double b = (origin_point.x * origin_point.x - first_wheel_center.x * first_wheel_center.x + origin_point.y * origin_point.y - first_wheel_center.y * first_wheel_center.y - R * R + S * S) / (2 * (origin_point.y - first_wheel_center.y));
                double A = k * k + 1;
                double B = 2 * k * b - 2 * first_wheel_center.x - 2 * first_wheel_center.y * k;
                double C = first_wheel_center.x * first_wheel_center.x + first_wheel_center.y * first_wheel_center.y - 2 * first_wheel_center.y * b + b * b - S * S;
                int res = SquareEquation(A, B, C, roots);
                if(res == 0) {
                    double x2c = roots[0];
                    double y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps) {
                        second_wheel_center.x = x2c;
                        second_wheel_center.y = y2c;
                        break;
                    }
                } else if(res == 1) {
                    double x2c;
                    double y2c;
                    if(k * roots[0] + b > k * roots[1] + b) {
                        x2c = roots[0];
                    } else {
                        x2c = roots[1];
                    }
                    y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps) {
                        second_wheel_center.x = x2c;
                        second_wheel_center.y = y2c;
                        break;
                    }
                }
            }
        }
    }
    return second_wheel_center;
}
*/
