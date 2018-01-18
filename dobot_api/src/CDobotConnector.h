#ifndef CDOBOTCONNECTOR_H
#define CDOBOTCONNECTOR_H

#include <QObject>
#include <QStringList>

class QIODevice;

class CDobotConnector : public QObject
{
    Q_OBJECT
public:
    CDobotConnector();
    ~CDobotConnector();

    Q_INVOKABLE void onInit(void);

    Q_SIGNAL void newConnectStatus(bool connected);

    Q_SLOT void onBytesToWrite(const char *data, qint64 maxSize);
    Q_SIGNAL void bytesWritten(void);
    Q_SLOT void onBytesReady(void);
    Q_SIGNAL void bytesReady(QByteArray array);
private:
    Q_INVOKABLE void searchDobot(void *isFinishedAddr, void *resultAddr, void *dobotNameListAddr, unsigned int maxLen);
    Q_INVOKABLE void connectDobot(void *isFinishedAddr, void *resultAddr, void *portNameAddr, unsigned int baudrate, void *fwTypeAddr, void *versionAddr);
    Q_INVOKABLE void disconnectDobot(void *isFinishedAddr, void *resultAddr);
private:
    QStringList dobotPortNameList;
    QIODevice *ioDevice;
};

#endif // CDOBOTCONNECTOR_H
