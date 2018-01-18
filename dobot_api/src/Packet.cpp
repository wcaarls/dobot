/****************************************Copyright(c)*****************************************************
**                            Shenzhen Yuejiang Technology Co., LTD.
**
**                                 http://www.dobot.cc
**
**--------------File Info---------------------------------------------------------------------------------
** File name:           Packet.cpp
** Latest modified Date:2016-06-01
** Latest Version:      V2.0.0
** Descriptions:        Packet API
**
**--------------------------------------------------------------------------------------------------------
** Created by:
** Created date:        2016-06-01
** Version:             V1.0.0
** Descriptions:
**--------------------------------------------------------------------------------------------------------
*********************************************************************************************************/
#include "Packet.h"

/*********************************************************************************************************
** Function name:       PacketReadProcess
** Descriptions:        Packet read process
** Input parameters:    packetHandler
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
static void PacketReadProcess(ProtocolHandler *protocolHandler)
{
    RingBuffer *rxRawByteQueue = &protocolHandler->rxRawByteQueue;
    RingBuffer *rxPacketQueue = &protocolHandler->rxPacketQueue;
    Packet *packet = &protocolHandler->rxDLPacket;

    while (1) {
        uint8_t packetLen = RingBufferGetCount(rxRawByteQueue);
        /*
         * 2 sync bytes, 1 payload length, at least 2 payload(id, ctrl), and 1 checksum
         * That is, 4 extra bytes, and at least 2 payload
         */
        if (packetLen < 6) {
            break;
        }
        // The first sync byte
        uint8_t syncByte1 = *(uint8_t *)RingBufferDataAt(rxRawByteQueue, 0);
        if (syncByte1 != SYNC_BYTE) {
            RingBufferDequeue(rxRawByteQueue, 0);
            continue;
        }
        // The second sync byte
        uint8_t syncByte2 = *(uint8_t *)RingBufferDataAt(rxRawByteQueue, 1);
        if (syncByte2 != SYNC_BYTE) {
            RingBufferDequeue(rxRawByteQueue, 0);
            RingBufferDequeue(rxRawByteQueue, 0);
            continue;
        }
        // The payload length
        uint8_t payloadLen = *(uint8_t *)RingBufferDataAt(rxRawByteQueue, 2);
        if (payloadLen >= SYNC_BYTE) {
            RingBufferDequeue(rxRawByteQueue, 0);
            continue;
        }

        // If not enough data now, just break to wait
        if (packetLen < payloadLen + 4) {
            break;
        }
        // The payload will never larger than SYNC_BYTE
        if (packetLen >= SYNC_BYTE) {
            RingBufferDequeue(rxRawByteQueue, 0);
            continue;
        }

        // Checksum
        uint8_t checksum = 0;
        // payloadLen + 1, so we also plus the original checksum!!!
        for (uint32_t i = 0; i < (uint32_t)(payloadLen + 1); i++) {
            checksum += *(uint8_t *)RingBufferDataAt(rxRawByteQueue, 3 + i);
        }
        if (checksum != 0) {
            RingBufferDequeue(rxRawByteQueue, 0);
            RingBufferDequeue(rxRawByteQueue, 0);
            RingBufferDequeue(rxRawByteQueue, 0);
            continue;
        }

        // Now all are satisfied
        RingBufferDequeue(rxRawByteQueue, &packet->header.syncBytes[0]);
        RingBufferDequeue(rxRawByteQueue, &packet->header.syncBytes[1]);
        RingBufferDequeue(rxRawByteQueue, &packet->header.payloadLen);
        RingBufferDequeue(rxRawByteQueue, &packet->payload.id);
        RingBufferDequeue(rxRawByteQueue, &packet->payload.ctrl);
        for (uint32_t i = 0; i < (uint32_t)(payloadLen - 2); i++) {
            RingBufferDequeue(rxRawByteQueue, &packet->payload.params[i]);
        }
        RingBufferDequeue(rxRawByteQueue, &packet->checksum);
        // Enqueu to rxPacketQueue
        RingBufferEnqueue(rxPacketQueue, packet);
    }
}

/*********************************************************************************************************
** Function name:       PacketWriteProcess
** Descriptions:        Packet write process
** Input parameters:    packetHandler
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
static void PacketWriteProcess(ProtocolHandler *protocolHandler)
{
    RingBuffer *txPacketQueue = &protocolHandler->txPacketQueue;
    RingBuffer *txRawByteQueue = &protocolHandler->txRawByteQueue;

    while (RingBufferGetCount(txPacketQueue) != 0) {
        Packet *packet = (Packet *)RingBufferDataAt(txPacketQueue, 0);
        if (RingBufferGetLeft(txRawByteQueue) >= (uint32_t)(packet->header.payloadLen + 4)) {
            packet->header.syncBytes[0] = SYNC_BYTE;
            packet->header.syncBytes[1] = SYNC_BYTE;
            uint8_t checksum = 0;
            checksum += packet->payload.id;
            checksum += packet->payload.ctrl;
            for (uint32_t i = 0; i < (uint32_t)(packet->header.payloadLen - 2); i++) {
                checksum += packet->payload.params[i];
            }
            packet->checksum = (uint8_t)(0 - checksum);

            RingBufferEnqueue(txRawByteQueue, &packet->header.syncBytes[0]);
            RingBufferEnqueue(txRawByteQueue, &packet->header.syncBytes[1]);
            RingBufferEnqueue(txRawByteQueue, &packet->header.payloadLen);

            RingBufferEnqueue(txRawByteQueue, &packet->payload.id);
            RingBufferEnqueue(txRawByteQueue, &packet->payload.ctrl);
            for (uint32_t i = 0; i < (uint32_t)(packet->header.payloadLen - 2); i++) {
                RingBufferEnqueue(txRawByteQueue, &packet->payload.params[i]);
            }
            RingBufferEnqueue(txRawByteQueue, &packet->checksum);
            RingBufferDequeue(txPacketQueue, 0);
        } else {
            break;
        }
    }
}

/*********************************************************************************************************
** Function name:       PacketProcess
** Descriptions:        Packet write process
** Input parameters:    packetHandler
** Output parameters:   None
** Returned value:      None
*********************************************************************************************************/
void PacketProcess(ProtocolHandler *protocolHandler)
{
    PacketReadProcess(protocolHandler);
    PacketWriteProcess(protocolHandler);
}
