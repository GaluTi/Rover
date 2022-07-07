#include "mainwindow.h"
#include "ui_mainwindow.h"

//Деление строки по пробелам
std::vector<std::string> Split(std::string str) {
    std::vector<std::string> result;
    std::string current_string = "";

    for(unsigned int i = 0 ; i < str.length(); ++i) {
        if(str[i] != ' ') {
            current_string.push_back(str[i]);
        } else {
            result.push_back(current_string);
            current_string = "";
        }
    }
    result.push_back(current_string);
    return result;
}

//Параллельный перенос отрезка перпендикулярно вверх на h
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
    Vector2D result(x, y);
    return result;
}

//Решение квадратного уравнения Ax^2 + Bx + C = 0; возвращает 1 - две корня, 0 - один корень, -1 - нет корней
int SquareEquation(double A, double B, double C, std::vector<double> &roots) {
    roots.clear();
    double D = B * B - 4 * A * C;
    if(D < -eps) {
        roots.push_back(-1);
        return -1;
    }
    else if(D > eps) {
        roots.push_back(((-B) + sqrt(D)) / (2 * A));
        roots.push_back(((-B) - sqrt(D)) / (2 * A));
        return 1;
    }
    else {
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
    }
    else {
        return false;
    }
}

//Проверка пренадлежности точки верхней дуге окружности
bool IsOnTopSector(Vector2D sector_begin_point, Vector2D sector_end_point, Vector2D origin, Vector2D point) {
    if(fabs((origin.x - point.x) * (origin.x - point.x) + ((origin.y - point.y) * (origin.y - point.y)) - (sector_begin_point - origin).length() * (sector_begin_point - origin).length()) < eps && point.y > std::min(sector_begin_point.y, sector_end_point.y)) {
        return true;
    }
    else {
        return false;
    }
}

//Проверка пренадлежности точки линии центров колеса
bool IsOnWheelCenterLine(const std::vector<std::pair<Vector2D, Vector2D>> &wheel_center_line, Vector2D wheel_center_point, int &wheel_center_line_num) {
    Vector2D line_begin;
    Vector2D line_end;
    Vector2D origin;
    int check = 0;
    for(unsigned int i = 1; i < wheel_center_line.size() - 1; ++i) {
        line_begin = wheel_center_line[i].first;
        line_end = wheel_center_line[i + 1].first;
        origin = wheel_center_line[i].second;
        if(wheel_center_line[i].second != wheel_center_line[i + 1].second) {
            if(IsOnLine(line_begin, line_end, wheel_center_point)) {
                check = 2;
                wheel_center_line_num = i;
                break;
            }
        }
        else if(wheel_center_line[i].second == wheel_center_line[i + 1].second) {
            if(IsOnTopSector(line_begin, line_end, origin, wheel_center_point)) {
                check = 2;
                wheel_center_line_num = i;
                break;
            }
        }
    }
    if(check != 2) {
        return false;
    }
    return true;
}

//Поворот вектора на заданный угол(положительный угол -  против часовой стрелки)
Vector2D VectorRotate(Vector2D orig_vector, float angle) {
    Vector2D result;
    result.x = cos(angle) * orig_vector.x + sin(angle) * orig_vector.y;
    result.y = -sin(angle) * orig_vector.x + cos(angle) * orig_vector.y;
    return result;
}

//Вычисление вершины равнобедренного треугольника по точкам основания, углу при основании и длине ребра
Vector2D IsoscelesTriangleVertex(Vector2D left_point, Vector2D right_point, float angle) {
    Vector2D base_edge(left_point, right_point); //Вектор основания
    Vector2D left_edge = VectorRotate(base_edge, angle); //Левое ребро
    Vector2D m_base_edge(right_point, left_point); //Минус вектор основания
    Vector2D right_edge = VectorRotate(m_base_edge, -angle); //Правое ребро
    Vector2D vertex = CrossPoint(left_point, left_point + left_edge, right_point, right_point + right_edge);
    return vertex;
}

//Скалярное произведение векторов
double CrossProduct(Vector2D a, Vector2D b) {
    return a.x * b.y - a.y * b.x;
}

//Функция линии центров колеса, по координате x воозвращает координату у
float WheelCenterLine(const std::vector<std::pair<Vector2D, Vector2D>> &wheel_center_line, float x, int &line_num) {
    float y = 0;
    if(x < wheel_center_line[0].first.x){
        y = wheel_center_line[0].first.y;
        line_num = -1;
    }
    else if(x > wheel_center_line.back().first.x){
        y = wheel_center_line.back().first.y;
        line_num = -1;
    }
    else {
        for(unsigned int i = 0; i < wheel_center_line.size() - 1; ++i){
            Vector2D line_begin = wheel_center_line[i].first;
            Vector2D line_end = wheel_center_line[i + 1].first;
            Vector2D origin = wheel_center_line[i].second;
            if(line_begin.x <= x && x <= line_end.x){
                if(wheel_center_line[i].second != wheel_center_line[i + 1].second){
                    float k = (line_end.y - line_begin.y) / (line_end.x - line_begin.x);
                    float b = (line_begin.y * line_end.x - line_end.y * line_begin.x) / (line_end.x - line_begin.x);
                    y = k * x + b;
                    line_num = i;
                    break;
                }
                else if(wheel_center_line[i].second == wheel_center_line[i + 1].second) {
                    y = std::sqrt(R * R - (origin.x - x) * (origin.x - x)) + origin.y;
                    line_num = i;
                    break;
                }
            }
        }
    }
    return y;
}

//Проверка корректности заданной на поверхности точки. Если точка корректна, в последние две переменные на вход будут записаны координаты соответствующей точки на линии центров колеса и индекс начала участка линии центров колеса, на котором она находится
bool IsCorrectSurfacePoint(const std::vector<Vector2D> &surface, const std::vector<std::pair<Vector2D, Vector2D>> &wheel_center_line, Vector2D point, Vector2D &wheel_center_point, int &wheel_center_line_num) {
    //Проверка, что точка лежит на поверхности
    int check = 0;
    Vector2D line_begin;
    Vector2D line_end;
    Vector2D origin;
    for(unsigned int i = 1; i < surface.size() - 1; ++i) {
        line_begin = surface[i];
        line_end = surface[i + 1];
        if(IsOnLine(line_begin, line_end, point)) {
            check = 1;
            wheel_center_point = ParallelTransfer(line_begin, point, R).second;
            break;
        }
    }
    if(check == 0) {
        return false;
    }

    //Проверка, что центр колеса, расположенного в данной точке поверхности, попадает на линию центров колеса
    for(unsigned int i = 1; i < wheel_center_line.size() - 1; ++i) {
        line_begin = wheel_center_line[i].first;
        line_end = wheel_center_line[i + 1].first;
        origin = wheel_center_line[i].second;
        if(wheel_center_line[i].second != wheel_center_line[i + 1].second) {
            if(IsOnLine(line_begin, line_end, wheel_center_point)) {
                check = 2;
                wheel_center_line_num = i;
                break;
            }
        }
        else if(wheel_center_line[i].second == wheel_center_line[i + 1].second) {
            if(IsOnTopSector(line_begin, line_end, origin, wheel_center_point)) {
                check = 2;
                wheel_center_line_num = i;
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
Vector2D SecondWheelCenter(const std::vector<std::pair<Vector2D, Vector2D>> &wheel_center_line, Vector2D wheel_center_point, int wheel_center_line_num) {
    Vector2D first_wheel_center = wheel_center_point;
    Vector2D second_wheel_center;
    Vector2D line_begin;
    Vector2D line_end;
    int line_num =  wheel_center_line_num + 1;

    //Поиск координат центра второго колеса на линии центров колеса
    for(unsigned int i = line_num; i > 0; --i) {
        line_begin = wheel_center_line[i - 1].first;
        line_end = wheel_center_line[i].first;
        std::vector<double> roots;
        //Первый случай: рассматриваемый участок линии центров колеса - отрезок
        if(wheel_center_line[i].second != wheel_center_line[i - 1].second) {
            double k = (line_end.y - line_begin.y) / (line_end.x - line_begin.x);
            double b = (line_begin.y * line_end.x - line_end.y * line_begin.x) / (line_end.x - line_begin.x);
            double A = k * k + 1;
            double B = 2 * k * b - 2 * first_wheel_center.x - 2 * first_wheel_center.y * k;
            double C = first_wheel_center.x * first_wheel_center.x + first_wheel_center.y * first_wheel_center.y - 2 * first_wheel_center.y * b + b * b - S * S;
            int res = SquareEquation(A, B, C, roots);
            if(res == 0) {
                double x2c = roots[0];
                double y2c = k * x2c + b;
                if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x) {
                    second_wheel_center.x = x2c;
                    second_wheel_center.y = y2c;
                    break;
                }
            }
            else if(res == 1) {
                double x2c = roots[0];
                double y2c = k * x2c + b;
                if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x) {
                    second_wheel_center.x = x2c;
                    second_wheel_center.y = y2c;
                    break;
                }
                else {
                    x2c = roots[1];
                    y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x) {
                        second_wheel_center.x = x2c;
                        second_wheel_center.y = y2c;
                        break;
                    }
                }
            }
        }
        //Второй случай: рассматриваемый участок линии центров колеса - дуга окружности
        else {
            Vector2D origin_point = wheel_center_line[i].second;
            double k = - (origin_point.x - first_wheel_center.x) / (origin_point.y - first_wheel_center.y);
            double b = (origin_point.x * origin_point.x - first_wheel_center.x * first_wheel_center.x + origin_point.y * origin_point.y - first_wheel_center.y * first_wheel_center.y - R * R + S * S) / (2 * (origin_point.y - first_wheel_center.y));
            double A = k * k + 1;
            double B = 2 * k * b - 2 * first_wheel_center.x - 2 * first_wheel_center.y * k;
            double C = first_wheel_center.x * first_wheel_center.x + first_wheel_center.y * first_wheel_center.y - 2 * first_wheel_center.y * b + b * b - S * S;
            int res = SquareEquation(A, B, C, roots);
            if(res == 0) {
                double x2c = roots[0];
                double y2c = k * x2c + b;
                if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x && y2c > std::min(line_begin.y, line_end.y)) {
                    second_wheel_center.x = x2c;
                    second_wheel_center.y = y2c;
                    break;
                }
            }
            else if(res == 1) {
                double x2c = roots[0];
                double y2c = k * x2c + b;
                if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x && y2c > std::min(line_begin.y, line_end.y)) {
                    second_wheel_center.x = x2c;
                    second_wheel_center.y = y2c;
                    break;
                }
                else {
                    x2c = roots[1];
                    y2c = k * x2c + b;
                    if(line_begin.x - x2c < eps && x2c - line_end.x < eps && x2c < first_wheel_center.x && y2c > std::min(line_begin.y, line_end.y)) {
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

//Поиск координат второго центра подвески вариацией положения третьего колеса
Vector2D SecondSuspensionCenter(const std::vector<std::pair<Vector2D, Vector2D>> &wheel_center_line, Vector2D first_suspension_center, Vector2D second_wheel_center, Vector2D &third_wheel_center, Vector2D &fourth_wheel_center) {
    Vector2D second_suspension_center;
    Vector2D curr_third_wheel_center;
    Vector2D curr_fourth_wheel_center;
    bool while_check = false;
    int sign_check = -1;
    float step = -1;
    float size_eps = 0.05; //погрешность размера корпуса
    float dead_lock_eps = 1e-4; //критическое значение шага: если меньше, принудительно останавливаем алгоритм
    int step_nums = 0;
    int line_num = -1;
    float x = 0;
    //Выбор точки начала вариации. Если луноход резко меняет позицию, например, сброс системы из дальнего положения в начальное, то бужем вести поиск от второго колеса. При простом движении лунохода, новое положение третьего колеса будем искать около его положения в предыдущий момент времени.
    if((second_wheel_center - third_wheel_center).length() > 2 * suspension_distance || (third_wheel_center.x == 0 && third_wheel_center.y == 0)) {
        x = second_wheel_center.x;
    }
    else {
        x = third_wheel_center.x;
    }
    float y;
    //Варьируем положение третьего колеса, пока не получим расстояние между центрами подвески с заданной погрешностью, либо шаг не станет меньше допустимого
    while(!while_check) {
        x += step;
        y = WheelCenterLine(wheel_center_line, x, line_num);
        curr_third_wheel_center.x = x;
        curr_third_wheel_center.y = y;
        curr_fourth_wheel_center = SecondWheelCenter(wheel_center_line, curr_third_wheel_center, line_num);
        second_suspension_center = IsoscelesTriangleVertex(curr_third_wheel_center, curr_fourth_wheel_center, suspension_angle);
        Vector2D suspension_line = first_suspension_center - second_suspension_center;
        if((fabs(suspension_line.length() - suspension_distance) <= size_eps && second_wheel_center.x > curr_third_wheel_center.x) || fabs(step) < dead_lock_eps) {
            third_wheel_center = curr_third_wheel_center;
            fourth_wheel_center = curr_fourth_wheel_center;
            while_check = true;
        }
        else if(suspension_line.length() - suspension_distance >= size_eps && sign_check == -1 && fabs(step) >= dead_lock_eps) {
            step = -step / 2;
            sign_check = 1;
        }
        else if(suspension_line.length() - suspension_distance <= size_eps && sign_check == 1 && fabs(step) >= dead_lock_eps) {
            step = -step;
            sign_check = -1;
        }
         ++step_nums;
    }
    std::cout << step_nums << std::endl;
    return second_suspension_center;
}

//Расчёт координат точек частей подвески
std::vector<std::pair<Vector2D, Vector2D>> SuspensionPoints(Vector2D first_wheel_center, Vector2D second_wheel_center, Vector2D first_suspension_center, Vector2D third_wheel_center, Vector2D fourth_wheel_center, Vector2D second_suspension_center) {
    //Рассчёт точек первой оси подвески
    std::vector<std::pair<Vector2D, Vector2D>> suspension;
    Vector2D perpendicular = VectorRotate(first_wheel_center - second_wheel_center, M_PI/2);
    Vector2D left_point = first_wheel_center + perpendicular * (delta_suspension / perpendicular.length());
    Vector2D right_point = first_suspension_center + perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = second_wheel_center + perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = first_wheel_center - perpendicular * (delta_suspension / perpendicular.length());
    right_point = first_suspension_center - perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = second_wheel_center - perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    //Рассчёт точек второй оси подвески
    perpendicular = VectorRotate(third_wheel_center - fourth_wheel_center, M_PI/2);
    left_point = third_wheel_center + perpendicular * (delta_suspension / perpendicular.length());
    right_point = second_suspension_center + perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = fourth_wheel_center + perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = third_wheel_center - perpendicular * (delta_suspension / perpendicular.length());
    right_point = second_suspension_center - perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    left_point = fourth_wheel_center - perpendicular * (delta_suspension / perpendicular.length());
    suspension.push_back(std::make_pair(left_point, right_point));
    return suspension;
}

//Расчёт координат точек корпуса
std::vector<Vector2D> RoverBodyPoints(Vector2D first_suspension_center, Vector2D second_suspension_center) {
    std::vector<Vector2D> rover_body;
    Vector2D suspension_line(second_suspension_center, first_suspension_center);
    Vector2D perpendicular = VectorRotate(suspension_line, M_PI/2);
    Vector2D p_1 = second_suspension_center;
    Vector2D p_2 = first_suspension_center;
    Vector2D p_3 = p_2 - perpendicular * 0.5;
    Vector2D p_4 = p_1 - perpendicular * 0.5;
    rover_body.push_back(p_1);
    rover_body.push_back(p_2);
    rover_body.push_back(p_3);
    rover_body.push_back(p_4);
    return rover_body;
}
MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    time = 0.0;
    velocity = 0.01 * 1;
    this->mainTimer = new QTimer();
    connect(this->mainTimer, SIGNAL(timeout()), this, SLOT(onTimeChangedSlot()));
    ui->print_surface->setCheckState(Qt::Checked);
    ui->print_wheel_center_line->setCheckState(Qt::Checked);
    ui->print_rover->setCheckState(Qt::Unchecked);
    ui->position_slider->setMinimum(0);
    ui->position_slider->setMaximum(slider_size - 1);

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
    in.close();
    ui->draw_widget->surface = surface;
    ui->draw_widget->surface_calculated = true;
    ui->draw_widget->repaint();
    std::ofstream raport;
    raport.open("D:\\Programming\\Qt\\Rover\\raport.txt");
    if(raport.is_open()) {
        raport << "Surface input complete" << std::endl;
    }

    //Строим линию центров колеса
    std::vector<std::pair<Vector2D, Vector2D>> wheel_center_line;
    wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[0], surface[1], R).first, surface[0]));
    for(unsigned int i = 1; i < surface.size() - 1; ++i) {
        Vector2D line_1 = surface[i] - surface[i - 1];
        Vector2D line_2 = surface[i + 1] - surface[i];
        if(CrossProduct(line_1, line_2) >= eps) {
            std::pair<Vector2D, Vector2D> parallel_line_1 = ParallelTransfer(surface[i - 1], surface[i], R);
            std::pair<Vector2D, Vector2D> parallel_line_2 = ParallelTransfer(surface[i], surface[i + 1], R);
            wheel_center_line.push_back(std::make_pair(CrossPoint(parallel_line_1.first, parallel_line_1.second, parallel_line_2.first, parallel_line_2.second), surface[i]));
        }
        else if(CrossProduct(line_1, line_2) <= eps) {
            wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[i - 1], surface[i], R).second, surface[i]));
            wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[i], surface[i + 1], R).first, surface[i]));
        }
    }
    wheel_center_line.push_back(std::make_pair(ParallelTransfer(surface[surface.size() - 2], surface[surface.size() - 1], R).second, surface[surface.size() - 1]));
    ui->draw_widget->wheel_center_line = wheel_center_line;
    ui->draw_widget->wheel_center_line_calculated = true;
    ui->draw_widget->repaint();
    if(raport.is_open()) {
        raport << "Wheel center line complete" << std::endl;
    }

    //Задаём начальные условия
    current_wheel_center_line_begin = ui->draw_widget->wheel_center_line[1].first;
    current_wheel_center_line_end = ui->draw_widget->wheel_center_line[2].first;
    current_origin_1 = ui->draw_widget->wheel_center_line[1].second;
    current_origin_2 = ui->draw_widget->wheel_center_line[2].second;
    current_wheel_center_line_num = 1;

    //Вычисляем начально положение лунохода
    ui->draw_widget->first_wheel_center = current_wheel_center_line_begin;
    ui->draw_widget->first_wheel_calculated = true;
    ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, current_wheel_center_line_num);
    ui->draw_widget->second_wheel_calculated = true;

    ui->draw_widget->suspension.clear();
    ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
    ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
    ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
    ui->draw_widget->third_wheel_calculated = true;
    ui->draw_widget->fourth_wheel_calculated = true;
    ui->draw_widget->suspension_calculated = true;

    ui->draw_widget->rover_body.clear();
    ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
    ui->draw_widget->rover_body_calculated = true;

    //Записываем выходную информацию
    std::ofstream out;
    out.open("D:\\Programming\\Qt\\Rover\\output.txt");
    if(out.is_open()) {
        out << "Wheel center line:" << std::endl;
        for(unsigned int i = 0; i < wheel_center_line.size(); ++i) {
            out << wheel_center_line[i].first.x << " " << wheel_center_line[i].first.y << " | " << wheel_center_line[i].second.x << " " << wheel_center_line[i].second.y << std::endl;
        }
    }
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
    if(!this->mainTimer->isActive()) {
        this->mainTimer->setInterval(10);
        this->mainTimer->start();
        float x_length = ui->draw_widget->wheel_center_line[ui->draw_widget->wheel_center_line.size() - 2].first.x - ui->draw_widget->wheel_center_line[1].first.x;
        ui->position_slider->setValue((int)(ui->draw_widget->first_wheel_center.x * slider_size / x_length));
    }
    else {
        this->mainTimer->stop();
    }
}

void MainWindow::onTimeChangedSlot()
{
    //Первый случай: участок линии центров колеса - отрезок
    if(current_origin_1 != current_origin_2) {
        Vector2D line(current_wheel_center_line_begin, current_wheel_center_line_end);
        Vector2D origin_vector(current_wheel_center_line_begin, {current_wheel_center_line_begin.x + 10, current_wheel_center_line_begin.y});
        float alpha = acos((origin_vector * line) / (origin_vector.length() * line.length()));
        Vector2D current_position;
        if(CrossProduct(origin_vector, line) < eps) {
            alpha = -alpha;
        }
        Vector2D new_line_begin = VectorRotate(current_wheel_center_line_begin, alpha);
        float x_new = new_line_begin.x + velocity * time;
        float y_new = new_line_begin.y;
        current_position = VectorRotate({x_new, y_new}, -alpha);
        if(line.length() >= (current_position - current_wheel_center_line_begin).length() && IsOnLine(current_wheel_center_line_begin, current_wheel_center_line_end, current_position)) {
            ui->draw_widget->first_wheel_center = current_position;
            ui->draw_widget->first_wheel_calculated = true;
            ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, current_wheel_center_line_num);
            ui->draw_widget->second_wheel_calculated = true;

            ui->draw_widget->suspension.clear();
            ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
            ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
            ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
            ui->draw_widget->third_wheel_calculated = true;
            ui->draw_widget->fourth_wheel_calculated = true;
            ui->draw_widget->suspension_calculated = true;

            ui->draw_widget->rover_body.clear();
            ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
            ui->draw_widget->rover_body_calculated = true;
            ui->print_rover->setCheckState(Qt::Checked);
            ui->draw_widget->rover_check = true;

            ui->draw_widget->repaint();
            QString str = "Curren position\nx: %1\ny: %2\n";
            ui->current_data->setText(str.arg(ui->draw_widget->first_wheel_center.x).arg(ui->draw_widget->first_wheel_center.y));
            float x_length = ui->draw_widget->wheel_center_line[ui->draw_widget->wheel_center_line.size() - 2].first.x - ui->draw_widget->wheel_center_line[1].first.x;
            ui->position_slider->setValue((int)(current_position.x * slider_size / x_length));
        }
        else {
            if(current_wheel_center_line_num < ui->draw_widget->wheel_center_line.size() - 3) {
                current_wheel_center_line_num++;
                current_wheel_center_line_begin = ui->draw_widget->wheel_center_line[current_wheel_center_line_num].first;
                current_wheel_center_line_end = ui->draw_widget->wheel_center_line[current_wheel_center_line_num + 1].first;
                current_origin_1 = ui->draw_widget->wheel_center_line[current_wheel_center_line_num].second;
                current_origin_2 = ui->draw_widget->wheel_center_line[current_wheel_center_line_num + 1].second;
                time = 0.0;
            }
            else {
                this->mainTimer->stop();
            }
        }
    }

    //Второй случай: участок линии центров колеса - верхняя дуга окружности
    else {
        Vector2D sector_begin(current_origin_1, current_wheel_center_line_begin);
        Vector2D sector_end(current_origin_1, current_wheel_center_line_end);
        Vector2D origin(current_origin_1, {current_origin_1.x + 10, current_origin_1.y});
        float omega = velocity / R;
        float delta_angle = acos((sector_end * sector_begin) / (sector_end.length() * sector_begin.length())); //Угол между левой и правой точкой на окружности

        float delta_fi = omega * time;
        Vector2D current_position = VectorRotate(sector_begin, delta_fi) + current_origin_1;
        if(delta_angle >= delta_fi && IsOnTopSector(current_wheel_center_line_begin, current_wheel_center_line_end, current_origin_1, current_position)) {
            ui->draw_widget->start_point = current_origin_1;

            ui->draw_widget->first_wheel_center = current_position;
            ui->draw_widget->first_wheel_calculated = true;
            ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, current_wheel_center_line_num);
            ui->draw_widget->second_wheel_calculated = true;

            ui->draw_widget->suspension.clear();
            ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
            ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
            ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
            ui->draw_widget->third_wheel_calculated = true;
            ui->draw_widget->fourth_wheel_calculated = true;
            ui->draw_widget->suspension_calculated = true;

            ui->draw_widget->rover_body.clear();
            ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
            ui->draw_widget->rover_body_calculated = true;
            ui->print_rover->setCheckState(Qt::Checked);
            ui->draw_widget->rover_check = true;

            ui->draw_widget->repaint();
            QString str = "Curren position\nx: %1\ny: %2\n";
            ui->current_data->setText(str.arg(ui->draw_widget->first_wheel_center.x).arg(ui->draw_widget->first_wheel_center.y));
            float x_length = ui->draw_widget->wheel_center_line[ui->draw_widget->wheel_center_line.size() - 2].first.x - ui->draw_widget->wheel_center_line[1].first.x;
            ui->position_slider->setValue((int)(current_position.x * slider_size / x_length));
        }
        else {
            if(current_wheel_center_line_num < ui->draw_widget->wheel_center_line.size() - 3) {
                current_wheel_center_line_num++;
                current_wheel_center_line_begin = ui->draw_widget->wheel_center_line[current_wheel_center_line_num].first;
                current_wheel_center_line_end = ui->draw_widget->wheel_center_line[current_wheel_center_line_num + 1].first;
                current_origin_1 = ui->draw_widget->wheel_center_line[current_wheel_center_line_num].second;
                current_origin_2 = ui->draw_widget->wheel_center_line[current_wheel_center_line_num + 1].second;
                time = 0.0;
            }
            else {
                this->mainTimer->stop();
            }
        }
    }
    time += mainTimer->remainingTime();
}

//Отрисовка положения лунохода по начальной координате
void MainWindow::on_spawn_point_clicked()
{
    double x = ui->start_point_x->text().toDouble();
    double y = ui->start_point_y->text().toDouble();
    Vector2D start(x, y);
    int num = -1;
    if(IsCorrectSurfacePoint(ui->draw_widget->surface, ui->draw_widget->wheel_center_line, start, ui->draw_widget->first_wheel_center, num)) {
        ui->draw_widget->start_point.x = x;
        ui->draw_widget->start_point.y = y;
        ui->draw_widget->first_wheel_calculated = true;
        ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, num);
        ui->draw_widget->second_wheel_calculated = true;

        ui->draw_widget->suspension.clear();
        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
        ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
        ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
        ui->draw_widget->third_wheel_calculated = true;
        ui->draw_widget->fourth_wheel_calculated = true;
        ui->draw_widget->suspension_calculated = true;

        ui->draw_widget->rover_body.clear();
        ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
        ui->draw_widget->rover_body_calculated = true;
        ui->print_rover->setCheckState(Qt::Checked);
        ui->draw_widget->rover_check = true;

        ui->draw_widget->repaint();
        QString str = "Curren position\nx: %1\ny: %2\n";
        ui->current_data->setText(str.arg(ui->draw_widget->first_wheel_center.x).arg(ui->draw_widget->first_wheel_center.y));
        float x_length = ui->draw_widget->wheel_center_line[ui->draw_widget->wheel_center_line.size() - 2].first.x - ui->draw_widget->wheel_center_line[1].first.x;
        ui->position_slider->setValue((int)(ui->draw_widget->first_wheel_center.x * slider_size / x_length));
    } else {
        QMessageBox::warning(this, "Warning!", "Incorrect start point");
    }
    ui->start_point_x->clear();
    ui->start_point_y->clear();
}


void MainWindow::on_reset_point_clicked()
{
    time = 0.0;
    velocity = 0.01;
    current_wheel_center_line_begin = ui->draw_widget->wheel_center_line[1].first;
    current_wheel_center_line_end = ui->draw_widget->wheel_center_line[2].first;
    current_origin_1 = ui->draw_widget->wheel_center_line[1].second;
    current_origin_2 = ui->draw_widget->wheel_center_line[2].second;
    current_wheel_center_line_num = 1;
    if(this->mainTimer->isActive()) {
        this->mainTimer->stop();
    }

    ui->draw_widget->first_wheel_center = current_wheel_center_line_begin;
    ui->draw_widget->first_wheel_calculated = true;
    ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, current_wheel_center_line_num);
    ui->draw_widget->second_wheel_calculated = true;

    ui->draw_widget->suspension.clear();
    ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
    ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
    ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
    ui->draw_widget->third_wheel_calculated = true;
    ui->draw_widget->fourth_wheel_calculated = true;
    ui->draw_widget->suspension_calculated = true;

    ui->draw_widget->rover_body.clear();
    ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
    ui->draw_widget->rover_body_calculated = true;

    ui->print_rover->setCheckState(Qt::Unchecked);
    ui->position_slider->setValue(0);
    ui->draw_widget->repaint();
    ui->current_data->clear();
}

void MainWindow::on_print_surface_stateChanged(int arg1)
{
    if(arg1 == 0) {
        ui->draw_widget->surface_check = false;
        ui->draw_widget->repaint();
    }
    else {
        ui->draw_widget->surface_check = true;
        ui->draw_widget->repaint();
    }
}


void MainWindow::on_print_wheel_center_line_stateChanged(int arg1)
{
    if(arg1 == 0) {
        ui->draw_widget->wheel_center_line_check = false;
        ui->draw_widget->repaint();
    }
    else {
        ui->draw_widget->wheel_center_line_check = true;
        ui->draw_widget->repaint();
    }
}


void MainWindow::on_print_rover_stateChanged(int arg1)
{
    if(arg1 == 0) {
        ui->draw_widget->rover_check = false;
        ui->draw_widget->repaint();
    }
    else {
        ui->draw_widget->rover_check = true;
        ui->draw_widget->repaint();
    }
}


void MainWindow::on_position_slider_valueChanged(int value)
{
    if(ui->draw_widget->rover_check && ui->draw_widget->rover_body_calculated && !this->mainTimer->isActive()) {
        int line_num = -1;
        float x_length = ui->draw_widget->wheel_center_line[ui->draw_widget->wheel_center_line.size() - 2].first.x - ui->draw_widget->wheel_center_line[1].first.x;
        float x = x_length * value / slider_size;
        float y = WheelCenterLine(ui->draw_widget->wheel_center_line, x, line_num);
        current_wheel_center_line_begin = ui->draw_widget->wheel_center_line[line_num].first;
        current_wheel_center_line_end = ui->draw_widget->wheel_center_line[line_num + 1].first;
        current_origin_1 = ui->draw_widget->wheel_center_line[line_num].second;
        current_origin_2 = ui->draw_widget->wheel_center_line[line_num + 1].second;
        current_wheel_center_line_num = line_num;

        ui->draw_widget->first_wheel_center.x = x;
        ui->draw_widget->first_wheel_center.y = y;
        ui->draw_widget->first_wheel_calculated = true;
        ui->draw_widget->second_wheel_center = SecondWheelCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_wheel_center, line_num);
        ui->draw_widget->second_wheel_calculated = true;

        ui->draw_widget->suspension.clear();
        ui->draw_widget->first_suspension_center = IsoscelesTriangleVertex(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, suspension_angle);
        ui->draw_widget->second_suspension_center = SecondSuspensionCenter(ui->draw_widget->wheel_center_line, ui->draw_widget->first_suspension_center, ui->draw_widget->second_wheel_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center);
        ui->draw_widget->suspension = SuspensionPoints(ui->draw_widget->first_wheel_center, ui->draw_widget->second_wheel_center, ui->draw_widget->first_suspension_center, ui->draw_widget->third_wheel_center, ui->draw_widget->fourth_wheel_center, ui->draw_widget->second_suspension_center);
        ui->draw_widget->third_wheel_calculated = true;
        ui->draw_widget->fourth_wheel_calculated = true;
        ui->draw_widget->suspension_calculated = true;

        ui->draw_widget->rover_body.clear();
        ui->draw_widget->rover_body = RoverBodyPoints(ui->draw_widget->first_suspension_center, ui->draw_widget->second_suspension_center);
        ui->draw_widget->rover_body_calculated = true;

        ui->draw_widget->repaint();
        QString str = "Curren position\nx: %1\ny: %2\n";
        ui->current_data->setText(str.arg(ui->draw_widget->first_wheel_center.x).arg(ui->draw_widget->first_wheel_center.y));

    }
}
