#include "CDobotCommunicator.h"
#include <QThread>
#include <QDebug>

CDobotCommunicator::CDobotCommunicator()
    :QObject(0),
      connected(false),
      currentState(IdleState),
      leftSpace(0),
      timeout(3000)
{
    qRegisterMetaType<Message>("Message");
}

CDobotCommunicator::~CDobotCommunicator()
{

}

void CDobotCommunicator::onInit(void)
{
    qDebug() << metaObject()->className() << ":" << QThread::currentThread();

    messageHandlers.reserve(64);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(stateMachine()));
    timer->start(1);
}

void CDobotCommunicator::setCmdTimeout(void *isFinishedAddr, void *resultAddr, unsigned int timeout)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;

    this->timeout = timeout;

    *result = DobotCommunicate_NoError;
    *isFinished = true;
}

void CDobotCommunicator::insertMessage(void *isFinishedAddr, void *resultAddr, void *messageAddr)
{
    bool *isFinished = (bool *)isFinishedAddr;
    int *result = (int *)resultAddr;
    Message *message = (Message *)messageAddr;

    if (messageHandlers.count() > 128) {
        *result = DobotCommunicate_BufferFull;
        *isFinished = true;
        return;
    }
    MessageHandler messageHandler;
    messageHandler.isFinished = isFinished;
    messageHandler.result = result;
    messageHandler.message = message;

    messageHandlers.push_back(messageHandler);
}

void CDobotCommunicator::onNewConnectStatus(bool connected)
{
    this->connected = connected;
    if (!this->connected) {
        while (messageHandlers.isEmpty() == false) {
            *(messageHandlers.first().isFinished) = true;
            messageHandlers.removeFirst();
        }
        currentState = IdleState;
    }
    leftSpace = 0;
}

void CDobotCommunicator::onMessageReady(const Message &message)
{
    newMessage = message;
    isNewMessageArrived = true;
}

void CDobotCommunicator::stateMachine(void)
{
    static uint countLeftspaceTimeout = 0;
    if (!connected) {
        return;
    }
    while (true) {
        bool breakWhile = true;
        switch (currentState) {
            case IdleState:
                countLeftspaceTimeout = 0;
                if (messageHandlers.isEmpty()) {
                    break;
                }
                if (messageHandlers.at(0).message->isQueued &&
                    leftSpace == 0) {
                    // Get left space first
                    currentState = SendingGetLeftSpaceCmdState;
                    breakWhile = false;
                } else {
                    // Sending the command directly
                    currentState = SendingCmdState;
                    breakWhile = false;
                }
            break;

            case SendingGetLeftSpaceCmdState:
                do {
                    Message message;
                    message.id = ProtocolQueuedCmdLeftSpace;
                    message.rw = 0;
                    message.isQueued = false;
                    message.paramsLen = 0;

                    isNewMessageArrived = false;
                    emit messageToSend(message);
                } while (0);
                currentState = WaitLeftSpaceAckState;
                execTime.start();
            break;

            case WaitLeftSpaceAckState:
                if (isNewMessageArrived) {
                    if (newMessage.id != ProtocolQueuedCmdLeftSpace) {
                        currentState = SendingGetLeftSpaceCmdState;// Re-read
                        break;
                    }
                    memcpy(&leftSpace, &newMessage.params[0], sizeof(uint32_t));
                    if (leftSpace == 0) {
                        *(messageHandlers.at(0).result) = DobotCommunicate_BufferFull;
                        *(messageHandlers.at(0).isFinished) = true;
                        messageHandlers.removeFirst();
                        currentState = IdleState;
                    } else {
                        currentState = SendingCmdState;
                    }
                } else if (execTime.elapsed() > (int)timeout) {
                    if(++countLeftspaceTimeout > 3) {
                        *(messageHandlers.at(0).result) = DobotCommunicate_Timeout;
                        *(messageHandlers.at(0).isFinished) = true;
                        messageHandlers.removeFirst();
                        currentState = IdleState;
                    } else {
                        currentState = SendingGetLeftSpaceCmdState;// Re-read
                    }
                    break;
                }
            break;

            case SendingCmdState:
                isNewMessageArrived = false;
                emit messageToSend(*messageHandlers.at(0).message);
                currentState = WaitCmdAckState;
                execTime.start();
            break;

            case WaitCmdAckState:
                if (isNewMessageArrived) {
                    Message *message = messageHandlers.at(0).message;
                    if (newMessage.id != message->id) {
                        currentState = SendingCmdState;// Re-read
                        break;
                    }
                    if (message->isQueued) {
                        leftSpace--;
                    }
                    memcpy(message, &newMessage, sizeof(Message));
                    *(messageHandlers.at(0).result) = DobotCommunicate_NoError;
                    *(messageHandlers.at(0).isFinished) = true;
                    messageHandlers.removeFirst();

                    currentState = IdleState;
                } else if (execTime.elapsed() > (int)timeout) {
                    *(messageHandlers.at(0).result) = DobotCommunicate_Timeout;
                    *(messageHandlers.at(0).isFinished) = true;
                    messageHandlers.removeFirst();
                    currentState = IdleState;
                }
            break;
        }
        if (breakWhile) {
            break;
        }
    }
}
