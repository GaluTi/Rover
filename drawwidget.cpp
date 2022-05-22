#include "drawwidget.h"
#include <QPainter>
#include <QMouseEvent>
#include <QPen>
#include <math.h>

const int rz = 15;

DrawWidget::DrawWidget(QWidget *parent) : QWidget(parent)
{
    first_wheel = false;
    second_wheel = false;
    third_wheel = false;
    fourth_wheel = false;
    is_suspension = false;
    is_rover_body =false;
}

void DrawWidget::paintEvent(QPaintEvent *event) {
    int W = width() / 10;
    int H = 3 * height() / 4;

    QPainter p(this);

    QPen pen(Qt::black, 2);
    p.setPen(pen);
    QFont font = p.font();
    font.setPointSize(font.pointSize() * 2.5);
    p.setFont(font);
    //p.drawArc(W/2 - 5, H/2 - 5, 10, 10, 0, 360 * 16);
    //p.drawLine(W/2, H/2, W/2, H/2 - 100);
    QString str = "A%1";
    for(unsigned int i = 0; i < surface.size() - 1; ++i) {
        Vector2D line_begin = surface[i];
        Vector2D line_end = surface[i + 1];
        p.drawLine(W + rz * line_begin.x, H - rz * line_begin.y, W + rz * line_end.x, H - rz * line_end.y);
        //p.drawText(W + rz * line_begin.x - 20, H - rz * line_begin.y + 45, str.arg(i + 1));
    }
    //p.drawText(W + rz * surface[surface.size() - 1].x - 20, H - rz * surface[surface.size() - 1].y + 45, str.arg(surface.size()));

    pen.setColor(Qt::red);
    p.setPen(pen);
    str = "B%1";
    for(unsigned int i = 0; i < wheel_center_line.size() - 1; ++i) {
        Vector2D line_begin = wheel_center_line[i].first;
        Vector2D line_end = wheel_center_line[i + 1].first;
        if(wheel_center_line[i].second != wheel_center_line[i + 1].second) {    
            pen.setColor(Qt::red);
            p.setPen(pen);
            p.drawLine(W + rz * line_begin.x, H - rz * line_begin.y, W + rz * line_end.x, H - rz * line_end.y);
            //QPen pen(Qt::black, 6);
            //p.setPen(pen);
            //p.drawPoint(W + rz * line_begin.x, H - rz * line_begin.y);
            //p.drawText(W + rz * line_begin.x - 15, H - rz * line_begin.y - 20, str.arg(i + 1));
        }
        else {
            Vector2D origin = wheel_center_line[i].second;
            Vector2D origin_v(origin.x + R, origin.y);
            Vector2D origin_vector(origin, origin_v);
            Vector2D left_vector(origin, line_begin);
            Vector2D right_vector(origin, line_end);

            float start_angle = 16 * 180 / M_PI * acos((origin_vector * right_vector) / (origin_vector.length() * right_vector.length()));
            float end_angle = 16 * 180 / M_PI * acos((origin_vector * left_vector) / (origin_vector.length() * left_vector.length()));
            float delta_angle = 16 * 180 / M_PI * acos((right_vector * left_vector) / (right_vector.length() * left_vector.length()));

            pen.setColor(Qt::red);
            p.setPen(pen);
            p.drawArc(W + rz * (origin.x - R), H - rz * (origin.y + R), rz * 2 * R, rz * 2 * R, start_angle, delta_angle);
            //QPen pen(Qt::black, 6);
            //p.setPen(pen);
            //p.drawPoint(W + rz * line_begin.x, H - rz * line_begin.y);
            //p.drawText(W + rz * line_begin.x - 35, H - rz * line_begin.y - 20, str.arg(i + 1));
        }
        //QPen pen(Qt::black, 6);
        //p.setPen(pen);
        //p.drawPoint(W + rz * wheel_center_line[wheel_center_line.size() - 1].first.x, H - rz * wheel_center_line[wheel_center_line.size() - 1].first.y);
        //p.drawText(W + rz * wheel_center_line[wheel_center_line.size() - 1].first.x - 35, H - rz * wheel_center_line[wheel_center_line.size() - 1].first.y - 20, str.arg(wheel_center_line.size()));
    }

    pen.setColor(Qt::blue);
    pen.setWidth(2);
    p.setPen(pen);
    if(first_wheel) {
        p.drawArc(W + rz * (first_wheel_center.x - R), H - rz * (first_wheel_center.y + R), rz * 2 * R, rz * 2 * R, 0, 360 * 16);
        pen.setColor(Qt::black);
        pen.setWidth(4);
        p.setPen(pen);
        p.drawPoint(W + rz * (first_wheel_center.x), H - rz * (first_wheel_center.y));
    }

    pen.setColor(Qt::blue);
    pen.setWidth(2);
    p.setPen(pen);
    if(second_wheel) {
        p.drawArc(W + rz * (second_wheel_center.x - R), H - rz * (second_wheel_center.y + R), rz * 2 * R, rz * 2 * R, 0, 360 * 16);
        pen.setColor(Qt::black);
        pen.setWidth(4);
        p.setPen(pen);
        p.drawPoint(W + rz * (second_wheel_center.x), H - rz * (second_wheel_center.y));
    }

    pen.setColor(Qt::blue);
    pen.setWidth(2);
    p.setPen(pen);
    if(third_wheel) {
        p.drawArc(W + rz * (third_wheel_center.x - R), H - rz * (third_wheel_center.y + R), rz * 2 * R, rz * 2 * R, 0, 360 * 16);
        pen.setColor(Qt::black);
        pen.setWidth(4);
        p.setPen(pen);
        p.drawPoint(W + rz * (third_wheel_center.x), H - rz * (third_wheel_center.y));
    }

    pen.setColor(Qt::blue);
    pen.setWidth(2);
    p.setPen(pen);
    if(fourth_wheel) {
        p.drawArc(W + rz * (fourth_wheel_center.x - R), H - rz * (fourth_wheel_center.y + R), rz * 2 * R, rz * 2 * R, 0, 360 * 16);
        pen.setColor(Qt::black);
        pen.setWidth(4);
        p.setPen(pen);
        p.drawPoint(W + rz * (fourth_wheel_center.x), H - rz * (fourth_wheel_center.y));
    }

    pen.setColor(Qt::blue);
    pen.setWidth(1);
    p.setPen(pen);
    if(is_suspension) {
        for(unsigned int i = 0; i < suspension.size() - 1; i = i + 2) {
            p.drawLine(W + rz * suspension[i].first.x, H - rz * suspension[i].first.y, W + rz * suspension[i].second.x, H - rz * suspension[i].second.y);
            p.drawLine(W + rz * suspension[i + 1].first.x, H - rz * suspension[i + 1].first.y, W + rz * suspension[i + 1].second.x, H - rz * suspension[i + 1].second.y);
        }
        pen.setColor(Qt::black);
        p.setPen(pen);
        p.drawLine(W + rz * suspension[0].first.x, H - rz * suspension[0].first.y, W + rz * suspension[2].first.x, H - rz * suspension[2].first.y);
        p.drawLine(W + rz * suspension[1].first.x, H - rz * suspension[1].first.y, W + rz * suspension[3].first.x, H - rz * suspension[3].first.y);
        p.drawLine(W + rz * suspension[0].second.x, H - rz * suspension[0].second.y, W + rz * suspension[2].second.x, H - rz * suspension[2].second.y);

        p.drawLine(W + rz * suspension[4].first.x, H - rz * suspension[4].first.y, W + rz * suspension[6].first.x, H - rz * suspension[6].first.y);
        p.drawLine(W + rz * suspension[5].first.x, H - rz * suspension[5].first.y, W + rz * suspension[7].first.x, H - rz * suspension[7].first.y);
        p.drawLine(W + rz * suspension[4].second.x, H - rz * suspension[4].second.y, W + rz * suspension[6].second.x, H - rz * suspension[6].second.y);

        pen.setWidth(4);
        p.setPen(pen);
        p.drawPoint(W + rz * first_suspension_center.x, H - rz * first_suspension_center.y);
        p.drawPoint(W + rz * second_suspension_center.x, H - rz * second_suspension_center.y);
    }

    pen.setColor(Qt::black);
    pen.setWidth(2);
    p.setPen(pen);
    if(is_rover_body) {
        p.drawLine(W + rz * rover_body[0].x, H - rz * rover_body[0].y, W + rz * rover_body[1].x, H - rz * rover_body[1].y);
        p.drawLine(W + rz * rover_body[1].x, H - rz * rover_body[1].y, W + rz * rover_body[2].x, H - rz * rover_body[2].y);
        p.drawLine(W + rz * rover_body[2].x, H - rz * rover_body[2].y, W + rz * rover_body[3].x, H - rz * rover_body[3].y);
        p.drawLine(W + rz * rover_body[3].x, H - rz * rover_body[3].y, W + rz * rover_body[0].x, H - rz * rover_body[0].y);
    }
}
