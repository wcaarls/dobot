#include "CDobotConnector.h"
#include <QSerialPort>
#include <QSerialPortInfo>
#include "DobotType.h"
#include "DobotDevice/UdpSearch.h"
#include "DobotDevice/UdpPort.h"
#include <QThread>
#include <QCoreApplication>

CDobotConnector::CDobotConnector()
    :QObject(0),
    ioDevice(NULL)
{

}

CDobotConnector::~CDobotConnector()
{

}

void CDobotConnector::onInit(void)
{
    qDebug() << metaObject()->className() << ":" << QThread::currentThread();
}

//Q_SIGNAL void newConnectStatus(bool connected, QIODevice *ioDevice);

void CDobotConnector::onBytesToWrite(const char *data, qint64 maxSize)
{
    if (ioDevice) {
        ioDevice->write(data, maxSize);
    }
}

//Q_SIGNAL void bytesWritten(void);

void CDobotConnector::onBytesReady(void)
{
    emit bytesReady(ioDevice->readAll());
}

//Q_SIGNAL void bytesReady(QByteArray data);


void CDobotConnector::searchDobot(void *isFinishedAddr, void *resultAddr, void *dobotNameListAddr, unsigned int maxLen)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    char *dobotNameList = (char *)dobotNameListAddr;

    QList<QSerialPortInfo> availablePorts = QSerialPortInfo::availablePorts();
    QStringList udpPortList = UdpSearch::Instance(this)->SearchDobotDevice();
    dobotPortNameList.clear();

    foreach(QSerialPortInfo portInfo, availablePorts) {
        if (portInfo.description().contains("USB-to-Serial")
            || portInfo.description().contains("CH340")
            || portInfo.description().contains("CP210")
            || portInfo.description().contains("USB2.0-Serial")
            || portInfo.description().contains("USB Serial Port")) {
            dobotPortNameList.push_back(portInfo.portName());
        }
    }
    // Now all the dobot informations are stored in dobotPortNameList
    do {
        if (dobotNameList == NULL) {
            break;
        }
        strcpy(dobotNameList, "");
        foreach(const QString &dobotStr, dobotPortNameList) {
            std::string stdString = dobotStr.toStdString();
            const char *portName = stdString.c_str();
            uint32_t nextLen = strlen(dobotNameList) + 1/*\0*/ + strlen(portName) + 1/* */;

            if (nextLen > maxLen) {
                break;
            }
            strcat(dobotNameList, portName);
            strcat(dobotNameList, " ");
        }

        for (int i = 0; i < udpPortList.size(); ++i){
            const char *idAddr = udpPortList.at(i).toLocal8Bit().constData();
            uint32_t nextLen = strlen(dobotNameList) + 1/*\0*/ + strlen(idAddr) + 1/* */;

            if (nextLen > maxLen) {
                break;
            }
            strcat(dobotNameList, idAddr);
            strcat(dobotNameList, " ");
        }

        // Trim the last blank
        if (dobotNameList[strlen(dobotNameList) - 1] == ' ') {
            dobotNameList[strlen(dobotNameList) - 1] = '\0';
        }
    } while (0);

    *result = dobotPortNameList.count() + udpPortList.count();
    *isFinished = true;
    return;
}

void CDobotConnector::connectDobot(void *isFinishedAddr, void *resultAddr, void *portNameAddr, unsigned int baudrate, void *fwTypeAddr, void *versionAddr)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;
    char *fwType = (char *)fwTypeAddr;
    char *version = (char *)versionAddr;

    const char *portName = (const char *)portNameAddr;

    QString connectPortName;
    if (portName == NULL || strcmp(portName, "") == 0) {
        bool _isFinished;
        int _result;
        searchDobot((void *)&_isFinished, (void *)&_result, 0, 0);
        if (dobotPortNameList.count() < 1) {
            *result = DobotConnect_NotFound;
            *isFinished = true;
            return;
        }
        connectPortName = dobotPortNameList.at(0);
    } else{
        connectPortName = QString(portName);
    }

    QRegExp regExpIP("((25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])[\\.]){3}(25[0-5]|2[0-4][0-9]|1[0-9][0-9]|[1-9][0-9]|[0-9])");
    if (!regExpIP.exactMatch(connectPortName)) {
        QSerialPort *serialPort = new QSerialPort(connectPortName, this);
        serialPort->setBaudRate(baudrate);
        serialPort->setDataBits(QSerialPort::Data8);
        serialPort->setParity(QSerialPort::NoParity);
        serialPort->setStopBits(QSerialPort::OneStop);
        ioDevice = serialPort;
    } else {
        if (fwType) {
            strcpy(fwType, "DobotWifi");
        }
        if (version) {
            strcpy(version, "0.0.0");
        }

        UdpPort *udpPort = new UdpPort(this);
        udpPort->setAddr(connectPortName);
        ioDevice = udpPort;
    }
    if (ioDevice->open(QIODevice::ReadWrite) == false) {
        ioDevice->deleteLater();
        ioDevice = 0;
        // Serial port is occupied!
        *result = DobotConnect_Occupied;
        *isFinished = true;
        return;
    }

    /****************************
     *
     * Check Room Type
     *
     ****************************/
    if (fwType && strcmp(fwType, "DobotWifi") != 0) {
        QRegExp regExpGRBL("GRBL\\:\\sV((\\d+\\.){2}\\d+)");
        QRegExp regExpMARLIN("MARLIN\\:\\sV((\\d+\\.){2}\\d+)");

        const uint32_t recvSize = 100;
        char recvData[recvSize];
        QString recvStr;

        ioDevice->write("\nM10\nM10\nM10\nM10\nM10\n");
        //ioDevice->flush();

        //Recv and check
        QDateTime startCheckTime(QDateTime::currentDateTime());
        QDateTime currentTime(startCheckTime);
        while(true) {
            if(startCheckTime.msecsTo(currentTime) > 500) {
                if (fwType) {
                    strcpy(fwType, "DobotSerial");
                }
                if (version) {
                    strcpy(version, "0.0.0");
                }
                break;
            }

            memset(recvData, 0, recvSize);
            if(ioDevice->read(recvData, recvSize) > 0) {
                recvStr += QString(recvData);

                if(recvStr.indexOf(regExpGRBL) >= 0) {
                    printf("GRBL\n");
                    if (fwType) {
                        strcpy(fwType, "GRBL");
                    }
                    if (version) {
                        strcpy(version, (const char*)regExpGRBL.cap(1).toLatin1().data());
                    }
                    ioDevice->close();
                    ioDevice->deleteLater();
                    ioDevice = NULL;
                    break;
                }

                if(recvStr.indexOf(regExpMARLIN) >= 0) {
                    printf("MARLIN\n");
                    if (fwType) {
                        strcpy(fwType, "MARLIN");
                    }
                    if (version) {
                        strcpy(version, (const char*)regExpMARLIN.cap(1).toLatin1().data());
                    }
                    ioDevice->close();
                    ioDevice->deleteLater();
                    ioDevice = NULL;
                    break;
                }
            }
            QCoreApplication::processEvents(QEventLoop::AllEvents, 10);
            currentTime = QDateTime::currentDateTime();
        }
    }
    if (ioDevice) {
        connect(ioDevice, SIGNAL(bytesWritten(qint64)), this, SIGNAL(bytesWritten()));
        connect(ioDevice, SIGNAL(readyRead()), this, SLOT(onBytesReady()));

        emit newConnectStatus(true);
    }
    *result = DobotConnect_NoError;
    *isFinished = true;
    return;
}

void CDobotConnector::disconnectDobot(void *isFinishedAddr, void *resultAddr)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    emit newConnectStatus(false);
    if (ioDevice) {
        ioDevice->close();
        ioDevice->deleteLater();
        ioDevice = 0;
    }

    *result = DobotConnect_NoError;
    *isFinished = true;
    return;
}
