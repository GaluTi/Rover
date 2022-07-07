#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#pragma once
#include "drawwidget.h"
#include <QMainWindow>
#include <QMessageBox>
#include <QTimer>
#include <QDateTime>
#include <iostream>
#include <fstream>

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

    QTimer* mainTimer;
    float time;
    float velocity;
    Vector2D current_wheel_center_line_begin;
    Vector2D current_wheel_center_line_end;
    Vector2D current_origin_1; //Здесь и далее за origin обозначаются точки, которые могут быть центром окружности
    Vector2D current_origin_2;
    unsigned int current_wheel_center_line_num;

private slots:
    void on_start_button_clicked();
    void onTimeChangedSlot();
    void on_spawn_point_clicked();
    void on_reset_point_clicked();
    void on_print_surface_stateChanged(int arg1);
    void on_print_wheel_center_line_stateChanged(int arg1);
    void on_print_rover_stateChanged(int arg1);
    void on_position_slider_valueChanged(int value);

private:
    Ui::MainWindow *ui;
};
#endif // MAINWINDOW_H
