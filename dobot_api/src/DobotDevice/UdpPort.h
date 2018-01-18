#ifndef UDPPORT_H
#define UDPPORT_H

#include <QIODevice>
#include <QUdpSocket>
#include <QTimer>

class UdpPort : public QIODevice
{
    Q_OBJECT

public:
    explicit UdpPort(QObject *parent = 0);
    ~UdpPort();

    bool open(OpenMode mode) Q_DECL_OVERRIDE;
    void close() Q_DECL_OVERRIDE;
    void setAddr(const QString &name);

    bool canReadLine() const Q_DECL_OVERRIDE;

    qint64 bytesAvailable() const Q_DECL_OVERRIDE;
    qint64 bytesToWrite() const Q_DECL_OVERRIDE;

    bool waitForReadyRead(int msecs) Q_DECL_OVERRIDE;
    bool waitForBytesWritten(int msecs) Q_DECL_OVERRIDE;

    qint64 readData(char *data, qint64 maxlen) Q_DECL_OVERRIDE;
    qint64 readLineData(char *data, qint64 maxlen) Q_DECL_OVERRIDE;
    qint64 writeData(const char *data, qint64 len) Q_DECL_OVERRIDE;

signals:
    void SetHeartBeartEmit(bool);

private slots:
    void processPendingDatagrams();
    void OnHeartBeatTime();
    void SetHeartBeatSignal(bool);

private:
    QUdpSocket *m_udpSocket;
    quint16 m_port;
    QHostAddress m_addr;

    bool m_canReadLine;
    bool m_writeFlag;

    bool    m_heartBeatSignal;
    QTimer *m_heartBeatTimer;
};

#endif // UDPPORT_H
