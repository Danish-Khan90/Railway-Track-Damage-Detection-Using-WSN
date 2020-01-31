/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2014 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 4: Ambient Temperature
*/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <qdebug.h>
#include <QLCDNumber>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    // Get all available COM Ports and store them in a QList.
    QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();

    // Read each element on the list, but
    // add only USB ports to the combo box.
    for (int i = 0; i < ports.size(); i++)
    {
        if (ports.at(i).portName.contains("USB"))
        {
            ui->comboBox_Interface->addItem(ports.at(i).portName.toLocal8Bit().constData());
        }
    }
    // Show a hint if no USB ports were found.
    if (ui->comboBox_Interface->count() == 0){
        ui->textEdit_Status->insertPlainText("No USB ports available.\nConnect a USB device and try again.");
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::changeEvent(QEvent *e)
{
    QMainWindow::changeEvent(e);
    switch (e->type())
    {
    case QEvent::LanguageChange:
        ui->retranslateUi(this);
        break;
    default:
        break;
    }
}

void MainWindow::on_pushButton_open_clicked()
{
    port.setQueryMode(QextSerialPort::EventDriven);
    port.setPortName("/dev/" + ui->comboBox_Interface->currentText());
    port.setBaudRate(BAUD115200);
    port.setFlowControl(FLOW_OFF);
    port.setParity(PAR_NONE);
    port.setDataBits(DATA_8);
    port.setStopBits(STOP_1);
    port.open(QIODevice::ReadWrite);

    if (!port.isOpen())
    {
        error.setText("Unable to open port!");
        error.show();
        return;
    }

    QObject::connect(&port, SIGNAL(readyRead()), this, SLOT(receive()));

    ui->pushButton_close->setEnabled(true);
    ui->pushButton_open->setEnabled(false);
    ui->comboBox_Interface->setEnabled(false);
}

void MainWindow::on_pushButton_close_clicked()
{
    if (port.isOpen())port.close();
    ui->pushButton_close->setEnabled(false);
    ui->pushButton_open->setEnabled(true);
    ui->comboBox_Interface->setEnabled(true);
}

void MainWindow::receive()
{
    static QString str;
        char ch;
        while (port.getChar(&ch))
        {
            str.append(ch);
            if (ch == '\n')     // End of line, start decoding
            {
                str.remove("\n", Qt::CaseSensitive);
                ui->textEdit_Status->append(str);

                if (str.contains("Clearing Track ID Status"))        /* clearing each track section status*/
                 {
                     ui->trackID2->display(0);
                     ui->trackID3->display(0);
                     ui->trackID4->display(0);
                     ui->trackID5->display(0);
                     ui->trackID6->display(0);
                     ui->track_status->display(0);
                     ui->lcdNumber_light->display(0);
                     ui->trackID2->setPalette(Qt::red);

                }

                if (str.contains("Train Arrival Detected ="))       /* Display of arrival detection*/
                {

                    double value;
                    QStringList list = str.split(QRegExp("\\s"));

                    qDebug() << "Str value: " << str;
                    if(!list.isEmpty())
                    {
                        qDebug() << "List size " << list.size();
                        for (int i=0; i < list.size(); i++)
                        {
                            qDebug() << "List value "<< i <<" "<< list.at(i);
                            if (list.at(i) == "=")
                            {
                                value = list.at(i+1).toDouble();
                                //adjust to Degrees
                                printf("%f\n",value);
                            }
                        }
                    }

                    qDebug() << "Var value " << QString::number(value);
                    ui->lcdNumber_light->display(value);
                }

                if (str.contains("Faulted Track ID = "))        /* Fault Track ID display on Faulted Mode ID box*/
                {

                    double track_status;
                    QStringList list = str.split(QRegExp("\\s"));

                    qDebug() << "Str value: " << str;
                    if(!list.isEmpty())
                    {
                        qDebug() << "List size " << list.size();
                        for (int i=0; i < list.size(); i++)
                        {
                            qDebug() << "List value "<< i <<" "<< list.at(i);
                            if (list.at(i) == "=")
                            {
                                track_status = list.at(i+1).toDouble();
                                //adjust to Degrees
                                printf("%f\n",track_status);
                            }
                        }
                    }

                    if(track_status == 2)                         /* Binary value changed to 1 on specific mote ID box*/
                        ui->trackID2->display(1);
                    else if(track_status == 3)
                        ui->trackID3->display(1);
                    else if(track_status == 4)
                        ui->trackID4->display(1);
                    else if(track_status == 5)
                        ui->trackID5->display(1);
                    else if(track_status == 6)
                        ui->trackID6->display(1);


                    qDebug() << "Var value " << QString::number(track_status);
                    ui->track_status->display(track_status);
                }



                this->repaint();    // Update content of window immediately
                str.clear();
            }
        }
}
