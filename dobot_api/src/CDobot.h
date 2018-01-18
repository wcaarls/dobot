#ifndef CDOBOT_H
#define CDOBOT_H

#include "CDobotConnector.h"
#include "CDobotProtocol.h"
#include "CDobotCommunicator.h"

class QCoreApplication;

class CDobot : public QObject
{
    Q_OBJECT
public:
    static CDobot *instance(void);
    void exec(void);
private:
    CDobot(QObject *parent = 0);
    ~CDobot();
public:
    CDobotConnector *connector;
    QThread *connectorTargetThread;
    CDobotProtocol *protocol;
    QThread *protocolTargetThread;
    CDobotCommunicator *communicator;
    QThread *communicatorTargetThread;
    QCoreApplication *app;
};

#endif // CDOBOT_H
