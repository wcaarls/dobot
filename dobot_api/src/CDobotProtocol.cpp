#include "CDobotProtocol.h"
#include <QIODevice>
#include <QTimer>
#include <QThread>
#include <QDebug>

#define RAW_BYTE_BUFFER_SIZE    1024
#define PACKET_BUFFER_SIZE  8

CDobotProtocol::CDobotProtocol()
  : QObject(0),
    connected(false),
    readyWrite(true),
    txBuffer(new uint8_t[256]),
    protocolHandler(new ProtocolHandler),
    txRawByteBuffer(new uint8_t[RAW_BYTE_BUFFER_SIZE]),
    rxRawByteBuffer(new uint8_t[RAW_BYTE_BUFFER_SIZE]),
    txPacketBuffer(new Packet[PACKET_BUFFER_SIZE]),
    rxPacketBuffer(new Packet[PACKET_BUFFER_SIZE])
{
    RingBufferInit(&protocolHandler->txRawByteQueue, &txRawByteBuffer[0], RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
    RingBufferInit(&protocolHandler->rxRawByteQueue, &rxRawByteBuffer[0], RAW_BYTE_BUFFER_SIZE, sizeof(uint8_t));
    RingBufferInit(&protocolHandler->txPacketQueue, &txPacketBuffer[0], PACKET_BUFFER_SIZE, sizeof(Packet));
    RingBufferInit(&protocolHandler->rxPacketQueue, &rxPacketBuffer[0], PACKET_BUFFER_SIZE, sizeof(Packet));
}

CDobotProtocol::~CDobotProtocol()
{

}

void CDobotProtocol::onInit(void)
{
    qDebug() << metaObject()->className() << ":" << QThread::currentThread();
    QTimer *timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(periodicTask()));
    timer->start(1);
}

void CDobotProtocol::onNewConnectStatus(bool connected)
{
    this->connected = connected;
    this->readyWrite = true;
}

//Q_SIGNAL void bytesToWrite(const char *data, qint64 maxSize);

void CDobotProtocol::onBytesWritten(void)
{
    readyWrite = true;
}

void CDobotProtocol::onBytesReady(QByteArray array)
{
    foreach(char data, array) {
        RingBufferEnqueue(&protocolHandler->rxRawByteQueue, &data);
    }
}

void CDobotProtocol::sendMessage(const Message &message)
{
    MessageWrite(&(*protocolHandler), &message);
}

//Q_SIGNAL void messageReady(const Message &message);

void CDobotProtocol::periodicTask(void)
{
    MessageProcess(&(*protocolHandler));

    if (readyWrite) {
        // Write
        uint32_t writeLen = 0;
        while (RingBufferIsEmpty(&protocolHandler->txRawByteQueue) == false) {
            RingBufferDequeue(&protocolHandler->txRawByteQueue, &txBuffer[writeLen]);
            writeLen++;
        }
        if (writeLen) {
            emit bytesToWrite((const char *)&txBuffer[0], writeLen);
            readyWrite = false;
        }
    }

    // Read
    Message message;
    while (MessageRead(&(*protocolHandler), &message) == ProtocolNoError) {
        emit messageReady(message);
    }
}
