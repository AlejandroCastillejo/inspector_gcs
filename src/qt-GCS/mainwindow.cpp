#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QFileDialog>
#include <QMessageBox>
#include <QTextStream>

QString qfilelocation;

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

void MainWindow::on_pushButton_OpenJsonFile_clicked()
{
    qfilelocation = QFileDialog::getOpenFileName(
                this,
                tr("Open File"),
                "~/home/",
                "Json files (*.json);; All files (*.*)"
                );
    QTextStream in(&qfilelocation);
    ui->textBrowser->setText(in.readAll());

//    QString filename = QFileDialog::getOpenFileName(
//                this,
//                tr("Open File"),
//                "~/home/",
//                "Json files (*.json);; All files (*.*)"
//                );

//    QMessageBox::information(this, tr("File Name"), filename);
}

void MainWindow::on_label_linkActivated(const QString &link)
{

}
