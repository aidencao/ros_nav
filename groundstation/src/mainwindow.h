#ifndef MAINWINDOW_H
#define MAINWINDOW_H
#include<stdio.h>
#include <QMainWindow>
#include <QFileDialog>
namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private slots:
    void on_refFile_clicked();
    void on_gpsSet_clicked();
    void on_resetHeight_clicked();
    void on_land_clicked();
    void on_takeoff_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
