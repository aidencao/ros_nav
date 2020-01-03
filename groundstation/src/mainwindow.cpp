#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "qt_app_node.h"
#include <iostream>
#include <ros/ros.h>
#include <string>
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}



void MainWindow::on_takeoff_clicked()
{
    send_take_off_cmd();
}

void MainWindow::on_land_clicked()
{
    send_land_cmd();
}

void MainWindow::on_resetHeight_clicked()
{
   QString strTxtEdt = ui->droneHeight->toPlainText();
   if(strTxtEdt.isEmpty())
   {
       ROS_WARN("set drone height is empty, please set value!");

   }
   else
   {
       float droneHeight = strTxtEdt.toFloat();
       send_drone_height_cmd(droneHeight);
   }

}

void MainWindow::on_gpsSet_clicked()
{
    QString numberTxtEdt = ui->gpsNumber->toPlainText();
    QString gpsHeightEdt = ui->gpsHeight->toPlainText();
    if(numberTxtEdt.isEmpty() || gpsHeightEdt.isEmpty())
    {
        ROS_WARN("gps point set is null, please set value!");
    }
    else
    {
        std::string str = numberTxtEdt.toStdString().append("$").append(gpsHeightEdt.toStdString());
        send_gps_height_and_number(str);
    }

}

void MainWindow::on_refFile_clicked()
{
    //system("gnome-terminal -x bash -c 'source ~/incubation_base/devel/setup.bash; roslaunch uav_communication uav_communication.launch'&");
   QFileDialog *dia = new QFileDialog;
   QString path =  dia->getOpenFileName(this);
   std::cout << path.toStdString() << std::endl;
   delete dia;
    //QString path =  QFileDialog::getOpenFileName(this);
   //std::cout << path.toStdString() << std::endl;
}
