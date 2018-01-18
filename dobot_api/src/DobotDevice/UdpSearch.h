#ifndef UDPSEARCH_H
#define UDPSEARCH_H

#include <QObject>
#include <QUdpSocket>
#include <QTimer>
#include <QTime>
#include <QMutex>
#include <QSignalMapper>

// the information of device
typedef class DeviceInfo {
public:
    DeviceInfo(): lifeTimeStamp(), localIP(){}
    DeviceInfo(QTime time, QString IP): lifeTimeStamp(time), localIP(IP){}

    // life time stamp
    QTime lifeTimeStamp;

    // local IP
    QString localIP;
}DEV_INFO;

class UdpSearch: public QObject
{
    Q_OBJECT
public:
    // get static UdpSearch object
    static UdpSearch  *Instance(QObject *parent = 0);

    // searching dobot and get the device list
    QStringList        SearchDobotDevice();

    // get the local IP connecting the device
    QString            GetLocalIP(QString &device);

    // heart beat
    void               Pacemaker(QString device);

private slots:
    // used to reading socket
    void               ProcessBroadCastDatagrams(QObject *object);

    // check device and network card
    void               OnCheckingTime();

private:
/******************************************************************************/
    UdpSearch(QObject *parent = 0);
    ~UdpSearch();

/******************************************************************************/
    // search local network card and modify m_socketMap
    inline void        SearchNetworkCard();

    // add device to m_deviceList
    inline void        DeviceListAdd(QString &deviceIP, QString localIP);

    // check device in m_deviceList is living or not
    inline void        DeviceLifeCheck();

private:
/******************************************************************************/
    // static UdpSearch object
    static UdpSearch          *INSTANCE;

    // port used for broadcast
    static const int           BROADCAST_PORT;

    // keyword used for broadcast
    static const QByteArray    BROADCAST_KEYWORD;

    // overtime of device life
    static const int           DEVICE_LIFE_OVERTIME;

    // local port
    static const int           LOCAL_PORT;

/******************************************************************************/
    // checking timer
    QTimer                    *m_checkingTimer;

    // mutex of socket list
    QMutex                     m_socketListMutex;

    // map of local IP and socket
    QMap<QString, QUdpSocket*> m_socketMap;

    // used for socket
    QSignalMapper             *m_signalMapper;

    // mutex of device list
    QMutex                     m_deviceListMutex;

    // map of device IP and device information
    QMap<QString, DEV_INFO>    m_deviceMap;
};

#endif // UDPSEARCH_H
