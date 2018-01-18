#include "UdpSearch.h"
#include <QtNetwork>

// init the static values
UdpSearch       *UdpSearch::INSTANCE             = NULL;
const int        UdpSearch::BROADCAST_PORT       = 48899;
const QByteArray UdpSearch::BROADCAST_KEYWORD    = "Who is Dobot?";
const int        UdpSearch::DEVICE_LIFE_OVERTIME = 5;
const int        UdpSearch::LOCAL_PORT           = 2046;

UdpSearch *UdpSearch::Instance(QObject *parent)
{
    if (INSTANCE == 0) {
        INSTANCE = new UdpSearch(parent);
    }

    return INSTANCE;
}

UdpSearch::UdpSearch(QObject *parent)
    : QObject(parent)
    , m_checkingTimer(NULL)
    , m_socketListMutex()
    , m_socketMap()
    , m_signalMapper(NULL)
    , m_deviceListMutex()
    , m_deviceMap()
{
    // clean up the containers
    m_socketMap.clear();
    m_deviceMap.clear();

    // setup signalMapper
    m_signalMapper = new QSignalMapper(this);
    connect(m_signalMapper, SIGNAL(mapped(QObject *)),
            this, SLOT(ProcessBroadCastDatagrams(QObject *)));

    // setup the timer
    m_checkingTimer = new QTimer(this);
    connect(m_checkingTimer, SIGNAL(timeout()), this, SLOT(OnCheckingTime()));
    m_checkingTimer->start(0);

    // search dobot via udp
    SearchDobotDevice();
}

UdpSearch::~UdpSearch()
{
    // release sockets
    foreach (QUdpSocket *socket, m_socketMap.values()) {
        if (socket) {
            socket->deleteLater();
            socket = NULL;
        }
    }

    // release timer
    if (m_checkingTimer) {
        m_checkingTimer->deleteLater();
        m_checkingTimer = NULL;
    }

    // release signalMapper
    if (m_signalMapper) {
        m_signalMapper->deleteLater();
        m_signalMapper = NULL;
    }
}

QStringList UdpSearch::SearchDobotDevice()
{
    //m_socketListMutex.lock();

    // send broadcast message
    foreach (QUdpSocket *socket, m_socketMap.values()) {
        socket->writeDatagram(BROADCAST_KEYWORD,
                              QHostAddress::Broadcast,
                              BROADCAST_PORT);

        //qDebug((QString("[") + QString::number(QTime::currentTime().second(), 10) + QString("S] Broadcast ") + QString(socket->localAddress().toString())).toLatin1().data());
    }

    //m_socketListMutex.unlock();

    // return device list as the result of last searching
    return m_deviceMap.keys();
}

void UdpSearch::OnCheckingTime()
{
    SearchNetworkCard();
    DeviceLifeCheck();

    m_checkingTimer->start(1000);
}

void UdpSearch::ProcessBroadCastDatagrams(QObject *object)
{
    //m_socketListMutex.lock();

    QUdpSocket *socket =  (QUdpSocket*)object;

    QByteArray   datagram("");
    QHostAddress senderIP("0.0.0.0");

    while (socket->hasPendingDatagrams()) {
        datagram.resize(socket->pendingDatagramSize());

        // read data
        qint64 res = socket->readDatagram(datagram.data(),
                                          datagram.size(),
                                         &senderIP,
                                          NULL);

        if (res != -1) { // success of reading data

            // change QHostAddress to String
            QString targetIP(senderIP.toString());
            // change datagram to String
            QString dataStr(datagram.data());

            // check IP is in the begining of datagram or not
            if (dataStr.indexOf(targetIP) == 0) {
                DeviceListAdd(targetIP, socket->peerAddress().toString());
            }
        }
    }

    //m_socketListMutex.unlock();
}

void UdpSearch::DeviceListAdd(QString &device, QString localIP)
{
    m_deviceListMutex.lock();
    if (m_deviceMap.keys().indexOf(device) == -1) {
        m_deviceMap.insert(device, DEV_INFO(QTime::currentTime(), localIP));
        //qDebug((QString("Add device: ") + device).toLatin1().data());
    } else {
        //qDebug((QString("###########Living: ") + device).toLatin1().data());
        m_deviceMap[device].lifeTimeStamp = QTime::currentTime();
    }
    m_deviceListMutex.unlock();
}

void UdpSearch::DeviceLifeCheck()
{
    QTime       currentTime(QTime::currentTime());  // current time stamp
    QStringList deviceList(m_deviceMap.keys());     // device list of map

    foreach (QString device, deviceList) {
        if (m_deviceMap[device].lifeTimeStamp.secsTo(currentTime) >
            DEVICE_LIFE_OVERTIME) {

            // lost device feeback over life overtime
            m_deviceListMutex.lock();
            m_deviceMap.remove(device);
            m_deviceListMutex.unlock();

            //qDebug((QString("!!!!!!!!!!!!!!!!!!!!!!!Lost device: ") + device).toLatin1().data());
        }
    }
}

QString UdpSearch::GetLocalIP(QString &device)
{
    return m_deviceMap[device].localIP;
}

void UdpSearch::SearchNetworkCard()
{
    //m_socketListMutex.lock();

    QHostInfo           info(QHostInfo::fromName(QHostInfo::localHostName()));
    QList<QHostAddress> addressList(info.addresses());
    QStringList         localIPList(m_socketMap.keys());

    // check old network card is living or not
    foreach(QString localIP, localIPList) {
        bool localIPLiving(false);
        foreach(QHostAddress address, addressList) {
            if (address.toString() == localIP) {
                localIPLiving = true;
                break;
            }
        }
        if (!localIPLiving) {
            // lost network card, release its socket
            if (m_socketMap[localIP] != NULL) {
                m_socketMap[localIP]->deleteLater();
                m_socketMap[localIP] = NULL;
            }

            //qDebug("lost network card");
            m_socketMap.remove(localIP);
        }
    }

    // check new network
    foreach(QHostAddress address, addressList) {
        if (address.protocol() != QAbstractSocket::IPv4Protocol) {
            continue;
        }

        bool isNewLocalIP(true);
        foreach (QString localIP, localIPList) {
            if (localIP == address.toString()) {
                isNewLocalIP = false;
                break;
            }
        }

        if (isNewLocalIP) {
            // new network card
            QUdpSocket *socket = new QUdpSocket(this);

            socket->bind(address,
                         LOCAL_PORT,
                         QUdpSocket::ShareAddress |
                         QUdpSocket::ReuseAddressHint);

            m_socketMap.insert(address.toString(), socket);

            m_signalMapper->setMapping(socket, (QObject*)socket);

            connect(socket, SIGNAL(readyRead()),
                    m_signalMapper, SLOT(map()));
        }
    }

    //m_socketListMutex.unlock();
}

void UdpSearch::Pacemaker(QString device)
{
    //qDebug((QString("\\\\\\\\\\\\\\\\Heart beat ") + device).toLatin1().data());
    m_deviceListMutex.lock();
    if (m_deviceMap.keys().indexOf(device) != -1) {
        m_deviceMap[device].lifeTimeStamp = QTime::currentTime();
    }
    m_deviceListMutex.unlock();
}
