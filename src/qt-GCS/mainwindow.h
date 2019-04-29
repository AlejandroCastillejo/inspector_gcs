#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>

//QString qfilelocation = QString::fromStdString("/home/alejandro/.ros/archivo2.json");
//QString qfilelocation;

namespace Ui {


class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:
    void on_pushButton_OpenJsonFile_clicked();

    void on_label_linkActivated(const QString &link);

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
