#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QMessageBox>
#include "drawwidget.h"
#include <fstream>

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

//Проверка пренадлежности точки верхней дуге окружности
bool IsOnSector(Vector2D sector_begin_point, Vector2D sector_end_point, Vector2D origin, Vector2D point) {
    if(fabs((origin.x - point.x) * (origin.x - point.x) + ((origin.y - point.y) * (origin.y - point.y)) - (sector_begin_point - origin).length() * (sector_begin_point - origin).length()) < eps && point.y - std::min(sector_begin_point.y, sector_end_point.y) > eps) {
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
Vector2D SecondWheelCenter(const std::vector<Vector2D> &_surface, const std::vector<std::pair<Vector2D, Vector2D>> &_wheel_center_line, Vector2D wheel_center_point, int wheel_center_line_num) {
    Vector2D first_wheel_center = wheel_center_point;
    Vector2D second_wheel_center;
    Vector2D line_begin;
    Vector2D line_end;
    int line_num =  wheel_center_line_num;

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
    return second_wheel_center;
}

double CrossProduct(Vector2D a, Vector2D b) {
    return a.x * b.y - a.y * b.x;
}

//Поворот вектора на заданный угол
Vector2D VectorRotate(Vector2D orig_vector, float angle) {
    Vector2D res_vector;
    res_vector.x = cos(angle) * orig_vector.x + sin(angle) * orig_vector.y;
    res_vector.y = -sin(angle) * orig_vector.x + cos(angle) * orig_vector.y;
    return res_vector;
}

//Вычисление вершины равнобедренного треугольника по двум точкам основания, углу и длине ребра
Vector2D IsoscelesTriangleVertex(Vector2D left_point, Vector2D right_point, float angle) {
    Vector2D base_edge(left_point, right_point);
    Vector2D left_edge = VectorRotate(base_edge, angle);
    Vector2D m_base_edge(right_point, left_point);
    Vector2D right_edge = VectorRotate(m_base_edge, -angle);
    Vector2D vertex = CrossPoint(left_point, left_point + left_edge, right_point, right_point + right_edge);
    return vertex;
}

//Нахождение начальной точки третьего колеса
Vector2D ThitdWheelCenter(const std::vector<Vector2D> &_surface, const std::vector<std::pair<Vector2D, Vector2D>> &_wheel_center_line, Vector2D second_wheel_center) {
    int line_num = -1;
    Vector2D third_wheel_center;
    for(unsigned int i = 0; i < _wheel_center_line.size() - 1; ++i) {
        Vector2D line_begin = _wheel_center_line[i].first;
        Vector2D line_end = _wheel_center_line[i + 1].first;
        if(_wheel_center_line[i].second != _wheel_center_line[i + 1].second) {
            if(IsOnLine(line_begin, line_end, second_wheel_center)) {
                line_num = i;
                break;
            }
        } else {
            Vector2D line_begin = _wheel_center_line[i].first;
            Vector2D line_end = _wheel_center_line[i + 1].first;
            Vector2D origin = _wheel_center_line[i].second;
            if(IsOnSector(line_begin, line_end, origin, second_wheel_center)) {
                line_num = i;
                break;
            }
        }
    }

    float len = S_T_W_D;
    int num = -1;
    if(line_num == -1) {
        return third_wheel_center;
    }

    for(unsigned int i = line_num; i > 0; --i) {
        Vector2D line_begin = _wheel_center_line[i].first;
        Vector2D line_end = _wheel_center_line[i + 1].first;
        Vector2D origin = _wheel_center_line[i].second;
        if(_wheel_center_line[i].second != _wheel_center_line[i + 1].second) {
            Vector2D line(second_wheel_center, line_begin);
            if(line.length() - len > eps) {
                Vector2D short_line = line * (len / line.length());
                third_wheel_center = short_line + second_wheel_center;
                num = i;
                break;
            } else {
                len = len - line.length();
                second_wheel_center = line_begin;
            }
        } else {
            Vector2D left_vector(origin, line_begin);
            Vector2D right_vector(origin, line_end);
            float full_sec_len = left_vector.length() * acos((left_vector * right_vector) / (left_vector.length() * right_vector.length()));
            if(full_sec_len - len > eps) {
                third_wheel_center = VectorRotate(right_vector, len / right_vector.length()) + origin;
                num = i;
                break;
            } else {
                len = len - full_sec_len;
                second_wheel_center = line_begin;
            }
        }
    }

    return third_wheel_center;
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    //Задаём поверхность
    std::vector<Vector2D> surface;
    std::string string_surface;

    std::ifstream in("D:\\Programming\\Qt\\Rover\\input.txt");
    if(in.is_open()) {
        while(getline(in, string_surface)) {
            std::vector<std::string> surface_points = Split(string_surface);
            Vector2D point = {std::stof(surface_points[0]), std::stof(surface_points[1])};
            surface.push_back(point);
        }
    }
    //std::cout << "Input complete" << std::endl;
    in.close();
    ui->draw_widget->surface = surface;
    ui->draw_widget->repaint();
    std::ofstream raport;
    raport.open("D:\\Programming\\Qt\\Rover\\raport.txt");
    if(raport.is_open()) {
        raport << "Surface input complete" << std::endl;
    }

    //Строим линию центра колёс
    std::vector<std::pair<Vector2D, Vector2D>> wheel_center_line;
    wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[0], surface[1], R).first, surface[0]));
    for(unsigned int i = 1; i < surface.size() - 1; ++i) {
        Vector2D line_1 = surface[i] - surface[i - 1];
        Vector2D line_2 = surface[i + 1] - surface[i];
        if(CrossProduct(line_1, line_2) > eps) {
            std::pair<Vector2D, Vector2D> parallel_line_1 = ParallelTransfer(surface[i - 1], surface[i], R);
            std::pair<Vector2D, Vector2D> parallel_line_2 = ParallelTransfer(surface[i], surface[i + 1], R);
            wheel_center_line.push_back(std::make_pair(CrossPoint(parallel_line_1.first, parallel_line_1.second, parallel_line_2.first, parallel_line_2.second), surface[i]));
        } else if(CrossProduct(line_1, line_2) < -eps) {
            wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[i - 1], surface[i], R).second, surface[i]));
            wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[i], surface[i + 1], R).first, surface[i]));
        }
    }
    wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[surface.size() - 2], surface[surface.size() - 1], R).second, surface[surface.size() - 1]));
    ui->draw_widget->wheel_center_line = wheel_center_line;
    ui->draw_widget->repaint();
    if(raport.is_open()) {
        raport << "Wheel center line complete" << std::endl;
    }

    //Что-то выводим
    std::ofstream out;
    out.open("D:\\Programming\\Qt\\Rover\\output.txt");
    if(out.is_open()) {
        out << "Wheel center line:" << std::endl;
        for(unsigned int i = 0; i < wheel_center_line.size(); ++i) {
            out << wheel_center_line[i].first.x << " " << wheel_center_line[i].first.y << " | " << wheel_center_line[i].second.x << " " << wheel_center_line[i].second.y << std::endl;
        }
    }
    //std::cout << "Output complete" << std::endl;
    out.close();
    if(raport.is_open()) {
        raport << "Output complete" << std::endl;
    }
    raport.close();

}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_start_button_clicked() {
    double x = ui->start_point_x->text().toDouble();
    double y = ui->start_point_y->text().toDouble();
    Vector2D start(x, y);
    int num = -1;
    if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, num)) {
        ui->draw_widget->start_point.x = x;
        ui->draw_widget->start_point.y = y;
        ui->draw_widget->first_wheel = true;

        ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, num);
        ui->draw_widget->second_wheel = true;

        ui->draw_widget->third_wheel_center = ThitdWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center);
        ui->draw_widget->third_wheel = true;

        //ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, num);
        //ui->draw_widget->third_wheel = true;

        ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, num);
        ui->draw_widget->fourth_wheel = true;

        ui->draw_widget->suspension.clear();
        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
        Vector2D first_perpendicular = VectorRotate(ui->draw_widget->first_wheel_center - ui->draw_widget->second_wheel_center, M_PI/2);
        Vector2D left_point = ui->draw_widget->first_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
        Vector2D right_point = ui->draw_widget->first_suspension_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->second_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->first_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
        right_point = ui->draw_widget->first_suspension_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->second_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));

        ui->draw_widget->second_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, suspension_angle);
        Vector2D second_perpendicular = VectorRotate(ui->draw_widget->third_wheel_center - ui->draw_widget->fourth_wheel_center, M_PI/2);
        left_point = ui->draw_widget->third_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
        right_point = ui->draw_widget->second_suspension_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->fourth_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->third_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
        right_point = ui->draw_widget->second_suspension_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        left_point = ui->draw_widget->fourth_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
        ui->draw_widget->is_suspension = true;

        ui->draw_widget->rover_body.clear();
        Vector2D suspension_line(ui->draw_widget->second_suspension_center, ui->draw_widget->first_suspension_center);
        Vector2D third_perpendicular = VectorRotate(suspension_line, M_PI/2);
        Vector2D p_1 = ui->draw_widget->second_suspension_center;
        Vector2D p_2 = ui->draw_widget->first_suspension_center;
        Vector2D p_3 = p_2 - third_perpendicular * 0.5;
        Vector2D p_4 = p_1 - third_perpendicular * 0.5;
        ui->draw_widget->rover_body.push_back(p_1);
        ui->draw_widget->rover_body.push_back(p_2);
        ui->draw_widget->rover_body.push_back(p_3);
        ui->draw_widget->rover_body.push_back(p_4);
        ui->draw_widget->is_rover_body = true;

        ui->draw_widget->repaint();
    } else {
        QMessageBox::warning(this, "Warning!", "Incorrect Start Point");
    }
    ui->start_point_x->clear();
    ui->start_point_y->clear();
}

/*void MainWindow::on_start_button_clicked() {
    for(unsigned int i = 0; i < ui->draw_widget->surface.size() - 1; ++i) {
        Vector2D line_begin = ui->draw_widget->surface[i];
        Vector2D line_end = ui->draw_widget->surface[i + 1];
        double k = (line_end.y - line_begin.y) / (line_end.x - line_begin.x);
        double b = (line_begin.y * line_end.x - line_end.y * line_begin.x) / (line_end.x - line_begin.x);
        for(unsigned int t = 0; t <= 500; t++) {
            double x = line_begin.x + (line_end.x - line_begin.x) * t/500;
            double y = k * x + b;
            Vector2D start(x, y);
            int line_num = -1;
            if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, line_num)) {
                ui->draw_widget->start_point.x = x;
                ui->draw_widget->start_point.y = y;

                ui->draw_widget->first_wheel = true;

                ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, line_num);
                ui->draw_widget->second_wheel = true;

                ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, line_num);
                ui->draw_widget->third_wheel = true;

                ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, line_num);
                ui->draw_widget->fourth_wheel = true;

                ui->draw_widget->repaint();

                ui->draw_widget->first_wheel = false;
                ui->draw_widget->second_wheel = false;
                ui->draw_widget->third_wheel = false;
                ui->draw_widget->fourth_wheel = false;
            }
        }
    }
}*/

/*void MainWindow::on_start_button_clicked() {
    float velocity = 0.1;
    for(unsigned int i = 0; i < ui->draw_widget->wheel_center_line.size() - 1; ++i) {
        if(ui->draw_widget->wheel_center_line[i].second != ui->draw_widget->wheel_center_line[i + 1].second) {
            Vector2D line_begin = ui->draw_widget->wheel_center_line[i].second;
            Vector2D line_end = ui->draw_widget->wheel_center_line[i + 1].second;
            double k = (line_end.y - line_begin.y) / (line_end.x - line_begin.x);
            double b = (line_begin.y * line_end.x - line_end.y * line_begin.x) / (line_end.x - line_begin.x);

            int T = (int)((line_end - line_begin).length() / velocity);
            Vector2D line(line_begin, line_end);
            Vector2D origin_vector(line_begin, {line_begin.x + 10, line_begin.y});
            float alpha = acos((origin_vector * line) / (origin_vector.length() * line.length()));

            double x;
            double y;
            if(CrossProduct(origin_vector, line) >= 0) {
                for(unsigned int t = 0; t <= T; t++) {
                    double x_new_0 = line_begin.x * cos(alpha) + line_begin.y * sin(alpha);
                    double y_new_0 = line_begin.y * cos(alpha) - line_begin.x * sin(alpha);
                    double x_new = x_new_0 + velocity * t;
                    double y_new = y_new_0;

                    x = x_new * cos(alpha) - y_new * sin(alpha);
                    y = x_new * sin(alpha) + y_new * cos(alpha);

                    Vector2D start(x, y);
                    int line_num = -1;
                    if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, line_num)) {
                        ui->draw_widget->start_point.x = x;
                        ui->draw_widget->start_point.y = y;

                        ui->draw_widget->first_wheel = true;

                        ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, line_num);
                        ui->draw_widget->second_wheel = true;

                        ui->draw_widget->third_wheel_center = ThitdWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center);
                        //ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, line_num);
                        ui->draw_widget->third_wheel = true;

                        ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, line_num);
                        ui->draw_widget->fourth_wheel = true;

                        ui->draw_widget->suspension.clear();
                        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
                        Vector2D first_perpendicular = VectorRotate(ui->draw_widget->first_wheel_center - ui->draw_widget->second_wheel_center, M_PI/2);
                        Vector2D left_point = ui->draw_widget->first_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        Vector2D right_point = ui->draw_widget->first_suspension_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->second_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->first_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        right_point = ui->draw_widget->first_suspension_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->second_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));

                        ui->draw_widget->second_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, suspension_angle);
                        Vector2D second_perpendicular = VectorRotate(ui->draw_widget->third_wheel_center - ui->draw_widget->fourth_wheel_center, M_PI/2);
                        left_point = ui->draw_widget->third_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        right_point = ui->draw_widget->second_suspension_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->fourth_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->third_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        right_point = ui->draw_widget->second_suspension_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->fourth_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        ui->draw_widget->is_suspension = true;

                        ui->draw_widget->rover_body.clear();
                        Vector2D suspension_line(ui->draw_widget->second_suspension_center, ui->draw_widget->first_suspension_center);
                        Vector2D third_perpendicular = VectorRotate(suspension_line, M_PI/2);
                        Vector2D p_1 = ui->draw_widget->second_suspension_center;
                        Vector2D p_2 = ui->draw_widget->first_suspension_center;
                        Vector2D p_3 = p_2 - third_perpendicular * 0.5;
                        Vector2D p_4 = p_1 - third_perpendicular * 0.5;
                        ui->draw_widget->rover_body.push_back(p_1);
                        ui->draw_widget->rover_body.push_back(p_2);
                        ui->draw_widget->rover_body.push_back(p_3);
                        ui->draw_widget->rover_body.push_back(p_4);
                        ui->draw_widget->is_rover_body = true;

                        ui->draw_widget->repaint();

                        //ui->draw_widget->first_wheel = false;
                        //ui->draw_widget->second_wheel = false;
                        //ui->draw_widget->third_wheel = false;
                        //ui->draw_widget->fourth_wheel = false;
                    }
                }
            } else {
                for(unsigned int t = 0; t <= T; t++) {
                    double x_new_0 = line_begin.x * cos(alpha) - line_begin.y * sin(alpha);
                    double y_new_0 = line_begin.y * cos(alpha) + line_begin.x * sin(alpha);
                    double x_new = x_new_0 + velocity * t;
                    double y_new = y_new_0;

                    x = x_new * cos(alpha) + y_new * sin(alpha);
                    y = -x_new * sin(alpha) + y_new * cos(alpha);

                    Vector2D start(x, y);
                    int line_num = -1;
                    if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, line_num)) {
                        ui->draw_widget->start_point.x = x;
                        ui->draw_widget->start_point.y = y;

                        ui->draw_widget->first_wheel = true;

                        ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, line_num);
                        ui->draw_widget->second_wheel = true;

                        ui->draw_widget->third_wheel_center = ThitdWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center);
                        //ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, line_num);
                        ui->draw_widget->third_wheel = true;

                        ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, line_num);
                        ui->draw_widget->fourth_wheel = true;

                        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
                        ui->draw_widget->is_suspension = true;

                        ui->draw_widget->suspension.clear();
                        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
                        Vector2D first_perpendicular = VectorRotate(ui->draw_widget->first_wheel_center - ui->draw_widget->second_wheel_center, M_PI/2);
                        Vector2D left_point = ui->draw_widget->first_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        Vector2D right_point = ui->draw_widget->first_suspension_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->second_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->first_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        right_point = ui->draw_widget->first_suspension_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->second_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));

                        ui->draw_widget->second_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, suspension_angle);
                        Vector2D second_perpendicular = VectorRotate(ui->draw_widget->third_wheel_center - ui->draw_widget->fourth_wheel_center, M_PI/2);
                        left_point = ui->draw_widget->third_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        right_point = ui->draw_widget->second_suspension_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->fourth_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->third_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        right_point = ui->draw_widget->second_suspension_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        left_point = ui->draw_widget->fourth_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                        ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                        ui->draw_widget->is_suspension = true;

                        ui->draw_widget->rover_body.clear();
                        Vector2D suspension_line(ui->draw_widget->second_suspension_center, ui->draw_widget->first_suspension_center);
                        Vector2D third_perpendicular = VectorRotate(suspension_line, M_PI/2);
                        Vector2D p_1 = ui->draw_widget->second_suspension_center;
                        Vector2D p_2 = ui->draw_widget->first_suspension_center;
                        Vector2D p_3 = p_2 - third_perpendicular * 0.5;
                        Vector2D p_4 = p_1 - third_perpendicular * 0.5;
                        ui->draw_widget->rover_body.push_back(p_1);
                        ui->draw_widget->rover_body.push_back(p_2);
                        ui->draw_widget->rover_body.push_back(p_3);
                        ui->draw_widget->rover_body.push_back(p_4);
                        ui->draw_widget->is_rover_body = true;

                        ui->draw_widget->repaint();

                        //ui->draw_widget->first_wheel = false;
                        //ui->draw_widget->second_wheel = false;
                        //ui->draw_widget->third_wheel = false;
                        //ui->draw_widget->fourth_wheel = false;
                    }
                }
            }

            /*Vector2D start(x, y);
            int line_num = -1;
            if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, line_num)) {
                ui->draw_widget->start_point.x = x;
                ui->draw_widget->start_point.y = y;

                ui->draw_widget->first_wheel = true;

                ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, line_num);
                ui->draw_widget->second_wheel = true;

                ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, line_num);
                ui->draw_widget->third_wheel = true;

                ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, line_num);
                ui->draw_widget->fourth_wheel = true;

                ui->draw_widget->repaint();

                //ui->draw_widget->first_wheel = false;
                //ui->draw_widget->second_wheel = false;
                //ui->draw_widget->third_wheel = false;
                //ui->draw_widget->fourth_wheel = false;
            }

        } else {
            Vector2D origin = ui->draw_widget->wheel_center_line[i].second;
            Vector2D sector_begin = ui->draw_widget->wheel_center_line[i].first;
            Vector2D sector_end = ui->draw_widget->wheel_center_line[i + 1].first;

            Vector2D left_vector(origin, sector_begin);
            Vector2D right_vector(origin, sector_end);

            float delta_angle = acos((right_vector * left_vector) / (right_vector.length() * left_vector.length()));

            float omega = velocity / R;
            int T = (int)((sector_end.x - sector_begin.x) / velocity);
            for(unsigned int t = 0; t <= T; t++) {
                double x = sector_begin.x + velocity * t;
                double y = sqrt(R * R - (origin.x - x) * (origin.x - x)) + origin.y;
                ui->draw_widget->first_wheel_center.x = x;
                ui->draw_widget->first_wheel_center.y = y;
                ui->draw_widget->first_wheel = true;

                ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, i);
                ui->draw_widget->second_wheel = true;

                ui->draw_widget->third_wheel_center = ThitdWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center);
                //ui->draw_widget->third_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->second_wheel_center, i);
                ui->draw_widget->third_wheel = true;

                ui->draw_widget->fourth_wheel_center = SecondWheelCenter(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, ui->draw_widget->third_wheel_center, i);
                ui->draw_widget->fourth_wheel = true;

                ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
                ui->draw_widget->is_suspension = true;

                ui->draw_widget->suspension.clear();
                ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
                Vector2D first_perpendicular = VectorRotate(ui->draw_widget->first_wheel_center - ui->draw_widget->second_wheel_center, M_PI/2);
                Vector2D left_point = ui->draw_widget->first_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                Vector2D right_point = ui->draw_widget->first_suspension_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->second_wheel_center + first_perpendicular * (delta_suspension / first_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->first_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                right_point = ui->draw_widget->first_suspension_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->second_wheel_center - first_perpendicular * (delta_suspension / first_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));

                ui->draw_widget->second_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, suspension_angle);
                Vector2D second_perpendicular = VectorRotate(ui->draw_widget->third_wheel_center - ui->draw_widget->fourth_wheel_center, M_PI/2);
                left_point = ui->draw_widget->third_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                right_point = ui->draw_widget->second_suspension_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->fourth_wheel_center + second_perpendicular * (delta_suspension / second_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->third_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                right_point = ui->draw_widget->second_suspension_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                left_point = ui->draw_widget->fourth_wheel_center - second_perpendicular * (delta_suspension / second_perpendicular.length());
                ui->draw_widget->suspension.push_back(std::make_pair(left_point, right_point));
                ui->draw_widget->is_suspension = true;

                ui->draw_widget->rover_body.clear();
                Vector2D suspension_line(ui->draw_widget->second_suspension_center, ui->draw_widget->first_suspension_center);
                Vector2D third_perpendicular = VectorRotate(suspension_line, M_PI/2);
                Vector2D p_1 = ui->draw_widget->second_suspension_center;
                Vector2D p_2 = ui->draw_widget->first_suspension_center;
                Vector2D p_3 = p_2 - third_perpendicular * 0.5;
                Vector2D p_4 = p_1 - third_perpendicular * 0.5;
                ui->draw_widget->rover_body.push_back(p_1);
                ui->draw_widget->rover_body.push_back(p_2);
                ui->draw_widget->rover_body.push_back(p_3);
                ui->draw_widget->rover_body.push_back(p_4);
                ui->draw_widget->is_rover_body = true;

                ui->draw_widget->repaint();

                //ui->draw_widget->first_wheel = false;
                //ui->draw_widget->second_wheel = false;
                //ui->draw_widget->third_wheel = false;
                //ui->draw_widget->fourth_wheel = false;
            }
        }
    }
}*/

