#include "UdpPort.h"
#include <QCoreApplication>
#include "UdpSearch.h"

UdpPort::UdpPort(QObject *parent):QIODevice(parent)
{
    m_udpSocket = NULL;
    m_canReadLine = false;
    m_writeFlag = false;
    m_heartBeatTimer = NULL;
    m_heartBeatSignal = false;
}

UdpPort::~UdpPort()
{
    if(m_udpSocket!= NULL) {
        m_udpSocket->deleteLater();
        m_udpSocket = NULL;
    }
}

void UdpPort::setAddr(const QString &name)
{
    m_addr = QHostAddress(name);
    m_port = 8899;
}

bool UdpPort::open(OpenMode mode)
{
    setOpenMode(mode);
    m_udpSocket = new QUdpSocket(this);
    m_udpSocket->open(QIODevice::ReadWrite);
    connect(m_udpSocket, SIGNAL(readyRead()), this, SLOT(processPendingDatagrams()));
    m_udpSocket->bind(QHostAddress(QHostAddress::AnyIPv4), 12346, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);

    m_heartBeatTimer = new QTimer(this);
    connect(m_heartBeatTimer, SIGNAL(timeout()), this, SLOT(OnHeartBeatTime()));
    m_heartBeatTimer->start(0);

    connect(this, SIGNAL(SetHeartBeartEmit(bool)), this, SLOT(SetHeartBeatSignal(bool)));

    return isOpen();
}

void UdpPort::close()
{
    setOpenMode(QIODevice::NotOpen);

    if (m_heartBeatTimer) {
        m_heartBeatTimer->deleteLater();
        m_heartBeatTimer = NULL;
    }

    if (m_udpSocket) {
        m_udpSocket->deleteLater();
        m_udpSocket = NULL;
    }
}

void UdpPort::OnHeartBeatTime()
{
    if (m_heartBeatSignal) {
        UdpSearch::Instance()->Pacemaker(m_addr.toString());
        emit SetHeartBeartEmit(false);
    }

    m_heartBeatTimer->start(2000);
}

void UdpPort::SetHeartBeatSignal(bool signal)
{
    m_heartBeatSignal = signal;
}

void UdpPort::processPendingDatagrams()
{
    if (m_udpSocket == NULL) {
        return;
    }

    if(m_udpSocket->hasPendingDatagrams()){
        emit SetHeartBeartEmit(true);
        m_canReadLine = true;
        emit readyRead();
    }
}

bool UdpPort::canReadLine() const
{
    return m_canReadLine;
}

qint64 UdpPort::bytesAvailable() const
{
    if (m_udpSocket == NULL) {
        return 0;
    }

    if(!m_udpSocket->hasPendingDatagrams()) {
        return 0;
    }

    return m_udpSocket->pendingDatagramSize();
}

qint64 UdpPort::bytesToWrite() const
{
    return 0;
}

bool UdpPort::waitForReadyRead(int msecs)
{
    if (m_udpSocket == NULL) {
        return false;
    }

    int slp = 0;
    while(!m_udpSocket->hasPendingDatagrams()){
        slp += 5;
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        if(slp >= msecs)
            return m_udpSocket->hasPendingDatagrams();
    }

    return m_udpSocket->hasPendingDatagrams();
}

bool UdpPort::waitForBytesWritten(int msecs)
{
    int slp = 0;
    while(!m_writeFlag){
        slp += 5;
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
        if(slp >= msecs)
            return m_writeFlag;
    }

    return m_writeFlag;
}

qint64 UdpPort::writeData(const char *data, qint64 len)
{
    if (m_udpSocket == NULL) {
        return 0;
    }

    m_writeFlag = false;
    qint64 siz = m_udpSocket->writeDatagram(data, len, m_addr, m_port);
    if(siz > 0){
        emit bytesWritten(siz);
    }
    m_writeFlag = true;
    return siz;
}

qint64 UdpPort::readData(char *data, qint64 maxlen)
{
    if (m_udpSocket == NULL) {
        return 0;
    }

    int ret = 0;
    if(m_udpSocket->hasPendingDatagrams())
    {
        QHostAddress sender;
        quint16 senderPort;
        ret = m_udpSocket->readDatagram(data, maxlen, &sender, &senderPort);
    }

    m_canReadLine = m_udpSocket->hasPendingDatagrams();
    return ret;
}

qint64 UdpPort::readLineData(char *data, qint64 maxlen)
{
    return readData(data, maxlen);
}


