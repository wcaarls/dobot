#include "DobotDll.h"
#include "dobotdll_global.h"
#include "CDobot.h"
#include <QThread>
#include <QCoreApplication>
#include <QDebug>

int DobotExec(void)
{
    CDobot::instance()->exec();

    return 0;
}

int SearchDobot(char *dobotNameList, uint32_t maxLen)
{
    QScopedPointer<bool> isFinished(new bool);
    QScopedPointer<int> result(new int);

    *isFinished = false;
    *result = 0;

    QMetaObject::invokeMethod(CDobot::instance()->connector,
                              "searchDobot",
                              Qt::QueuedConnection,
                              Q_ARG(void *, (void *)&(*isFinished)),
                              Q_ARG(void *, (void *)&(*result)),
                              Q_ARG(void *, (void *)dobotNameList),
                              Q_ARG(unsigned int, maxLen));

    while (*isFinished == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
    }
    return *result;
}

int ConnectDobot(const char *portName, uint32_t baudrate, char *fwType, char *version)
{
    QScopedPointer<bool> isFinished(new bool);
    QScopedPointer<int> result(new int);

    *isFinished = false;
    *result = 0;

    QMetaObject::invokeMethod(CDobot::instance()->connector,
                              "connectDobot",
                              Qt::QueuedConnection,
                              Q_ARG(void *, (void *)&(*isFinished)),
                              Q_ARG(void *, (void *)&(*result)),
                              Q_ARG(void *, (void *)portName),
                              Q_ARG(unsigned int, baudrate),
                              Q_ARG(void *, (void *)fwType),
                              Q_ARG(void *, (void *)version));

    while (*isFinished == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
    }
    return *result;
}

int DisconnectDobot(void)
{
    QScopedPointer<bool> isFinished(new bool);
    QScopedPointer<int> result(new int);

    *isFinished = false;
    *result = 0;

    QMetaObject::invokeMethod(CDobot::instance()->connector,
                              "disconnectDobot",
                              Qt::QueuedConnection,
                              Q_ARG(void *, (void *)&(*isFinished)),
                              Q_ARG(void *, (void *)&(*result)));

    while (*isFinished == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
    }
    return *result;
}

int SetCmdTimeout(unsigned int cmdTimeout)
{
    QScopedPointer<bool> isFinished(new bool);
    QScopedPointer<int> result(new int);

    *isFinished = false;
    *result = 0;

    QMetaObject::invokeMethod(CDobot::instance()->communicator,
                              "setCmdTimeout",
                              Qt::QueuedConnection,
                              Q_ARG(void *, (void *)&(*isFinished)),
                              Q_ARG(void *, (void *)&(*result)),
                              Q_ARG(unsigned int, cmdTimeout));
    while (*isFinished == false) {
        QCoreApplication::processEvents(QEventLoop::AllEvents, 5);
    }
    return *result;
}

#define WAIT_CMD_EXECUTION()                                            \
QScopedPointer<bool> isFinished(new bool);                              \
QScopedPointer<int> result(new int);                                    \
                                                                        \
*isFinished = false;                                                    \
*result = 0;                                                            \
                                                                        \
QMetaObject::invokeMethod(CDobot::instance()->communicator,             \
                          "insertMessage",                              \
                          Qt::QueuedConnection,                         \
                          Q_ARG(void *, (void *)isFinished.data()),     \
                          Q_ARG(void *, (void *)result.data()),         \
                          Q_ARG(void *, (void *)message.data()));       \
while (*isFinished == false) {                                          \
    QCoreApplication::processEvents(QEventLoop::AllEvents, 5);          \
}

/*********************************************************************************************************
** Device information
*********************************************************************************************************/
int SetDeviceSN(const char *deviceSN)
{
    // 0. Parameter checking
    if (deviceSN == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceSN;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    strcpy((char *)message.data()->params, (const char *)deviceSN);
    message.data()->paramsLen = strlen(deviceSN) + 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetDeviceSN(char *deviceSN, uint32_t maxLen)
{
    // 0. Parameter checking
    if (deviceSN == NULL ||
        maxLen == 0) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceSN;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    uint32_t len = strlen((const char *)&message.data()->params[0]);
    if (len < maxLen) {
        strcpy(deviceSN, (const char *)&message.data()->params[0]);
    } else {
        memcpy(deviceSN, (const char *)&message.data()->params[0], maxLen - 1);
        deviceSN[maxLen - 1] = 0;
    }

    return *result;
}

int SetDeviceName(const char *deviceName)
{
    // 0. Parameter checking
    if (deviceName == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceName;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    strcpy((char *)message.data()->params, deviceName);
    message.data()->paramsLen = strlen(deviceName) + 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetDeviceName(char *deviceName, uint32_t maxLen)
{
    // 0. Parameter checking
    if (deviceName == NULL ||
        maxLen == 0) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceName;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    uint32_t len = strlen((const char *)&message.data()->params[0]);
    if (len < maxLen) {
        strcpy(deviceName, (const char *)&message.data()->params[0]);
    } else {
        memcpy(deviceName, (const char *)&message.data()->params[0], maxLen - 1);
        deviceName[maxLen - 1] = 0;
    }

    return *result;
}

int GetDeviceVersion(uint8_t *majorVersion, uint8_t *minorVersion, uint8_t *revision)
{
    // 0. Parameter checking
    if (majorVersion == NULL ||
        minorVersion == NULL ||
        revision == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceVersion;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    *majorVersion = message.data()->params[0];
    *minorVersion = message.data()->params[1];
    *revision = message.data()->params[2];

    return *result;
}

int SetDeviceWithL(bool isWithL, bool isQueued, uint64_t *queuedCmdIndex)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->id = ProtocolDeviceWithL;

    message.data()->params[0] = isWithL;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetDeviceWithL(bool *isWithL)
{
    // 0. Parameter checking
    if (isWithL == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceWithL;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isWithL = (bool)message.data()->params[0];

    return *result;
}

int GetDeviceTime(uint32_t *deviceTime)
{
    // 0. Parameter checking
    if (deviceTime == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolDeviceTime;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(deviceTime, &message.data()->params[0], sizeof(uint32_t));

    return *result;
}

/*********************************************************************************************************
** Pose & Kinematics
*********************************************************************************************************/
int GetPose(Pose *pose)
{
    // 0. Parameter checking
    if (pose == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolGetPose;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(pose, &message.data()->params[0], sizeof(Pose));

    return *result;
}

int ResetPose(bool manual, float rearArmAngle, float frontArmAngle)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->id = ProtocolResetPose;

    message.data()->params[0] = manual;
    memcpy(&message.data()->params[1], &rearArmAngle, sizeof(float));
    memcpy(&message.data()->params[1 + sizeof(float)], &frontArmAngle, sizeof(float));

    message.data()->paramsLen = 1 + 2 * sizeof(float);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    return *result;
}

int GetKinematics(Kinematics *kinematics)
{
    // 0. Parameter checking
    if (kinematics == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolGetKinematics;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(kinematics, &message.data()->params[0], sizeof(Kinematics));

    return *result;
}

int GetPoseL(float *l)
{
    // 0. Parameter checking
    if (l == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolGetPoseL;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(l, &message.data()->params[0], sizeof(float));

    return *result;
}

/*********************************************************************************************************
** Alarms
*********************************************************************************************************/

int GetAlarmsState(uint8_t *alarmsState, uint32_t *len, unsigned int maxLen)
{
    // 0. Parameter checking
    if (alarmsState == NULL ||
        len == NULL ||
        maxLen == 0) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAlarmsState;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *len = message.data()->paramsLen;
    memcpy(alarmsState, &message.data()->params[0], *len > maxLen ? maxLen : *len);

    return *result;
}

int ClearAllAlarmsState(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAlarmsState;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    return *result;
}

/*********************************************************************************************************
** HOME
*********************************************************************************************************/
int SetHOMEParams(HOMEParams *homeParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (homeParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHOMEParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], homeParams, sizeof(HOMEParams));
    message.data()->paramsLen = sizeof(HOMEParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetHOMEParams(HOMEParams *homeParams)
{
    // 0. Parameter checking
    if (homeParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHOMEParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(homeParams, &message.data()->params[0], sizeof(HOMEParams));

    return *result;
}

int SetHOMECmd(HOMECmd *homeCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (homeCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHOMECmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], homeCmd, sizeof(HOMECmd));
    message.data()->paramsLen = sizeof(HOMECmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetAutoLevelingCmd(AutoLevelingCmd *autoLevelingCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (autoLevelingCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAutoLeveling;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], autoLevelingCmd, sizeof(AutoLevelingCmd));
    message.data()->paramsLen = sizeof(AutoLevelingCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetAutoLevelingResult(float *precision)
{
    // 0. Parameter checking
    if (precision == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAutoLeveling;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(precision, &message.data()->params[0], sizeof(float));

    return *result;
}

/*********************************************************************************************************
** HHT
*********************************************************************************************************/

int SetHHTTrigMode(HHTTrigMode hhtTrigMode)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHHTTrigMode;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->params[0] = hhtTrigMode;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetHHTTrigMode(HHTTrigMode *hhtTrigMode)
{
    // 0. Parameter checking
    if (hhtTrigMode == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHHTTrigMode;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *hhtTrigMode = (HHTTrigMode)message.data()->params[0];

    return *result;
}

int SetHHTTrigOutputEnabled(bool isEnabled)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHHTTrigOutputEnabled;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->params[0] = isEnabled;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetHHTTrigOutputEnabled(bool *isEnabled )
{
    // 0. Parameter checking
    if (isEnabled == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHHTTrigOutputEnabled;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isEnabled = message.data()->params[0];

    return *result;
}

int GetHHTTrigOutput(bool *isTriggered)
{
    // 0. Parameter checking
    if (isTriggered == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolHHTTrigOutput;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isTriggered = message.data()->params[0];

    return *result;
}

/*********************************************************************************************************
** End effector
*********************************************************************************************************/

int SetEndEffectorParams(EndEffectorParams *endEffectorParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (endEffectorParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], endEffectorParams, sizeof(EndEffectorParams));
    message.data()->paramsLen = sizeof(EndEffectorParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetEndEffectorParams(EndEffectorParams *endEffectorParams)
{
    // 0. Parameter checking
    if (endEffectorParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    memcpy(endEffectorParams, &message.data()->params[0], sizeof(EndEffectorParams));

    return *result;
}

int SetEndEffectorLaser(bool enableCtrl, bool on, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorLaser;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = enableCtrl;
    message.data()->params[1] = on;
    message.data()->paramsLen = 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetEndEffectorLaser(bool *isCtrlEnabled, bool *isOn)
{
    // 0. Parameter checking
    if (isCtrlEnabled == NULL ||
        isOn == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorLaser;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    *isCtrlEnabled = message.data()->params[0];
    *isOn = message.data()->params[1];

    return *result;
}

int SetEndEffectorSuctionCup(bool enableCtrl, bool suck, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorSuctionCup;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = enableCtrl;
    message.data()->params[1] = suck;
    message.data()->paramsLen = 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetEndEffectorSuctionCup(bool *isCtrlEnabled, bool *isSucked)
{
    // 0. Parameter checking
    if (isCtrlEnabled == NULL ||
        isSucked == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorSuctionCup;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    *isCtrlEnabled = message.data()->params[0];
    *isSucked = message.data()->params[1];

    return *result;
}

int SetEndEffectorGripper(bool enableCtrl, bool grip, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorGripper;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = enableCtrl;
    message.data()->params[1] = grip;
    message.data()->paramsLen = 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetEndEffectorGripper(bool *isCtrlEnabled, bool *isGripped)
{
    // 0. Parameter checking
    if (isCtrlEnabled == NULL ||
        isGripped == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEndEffectorGripper;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3. The result
    *isCtrlEnabled = message.data()->params[0];
    *isGripped = message.data()->params[1];

    return *result;
}

/*********************************************************************************************************
** Arm orientation
*********************************************************************************************************/

int SetArmOrientation(ArmOrientation armOrientation, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolArmOrientation;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = armOrientation;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetArmOrientation(ArmOrientation *armOrientation)
{
    // 0. Parameter checking
    if (armOrientation == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolArmOrientation;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *armOrientation = (ArmOrientation)message.data()->params[0];

    return *result;
}

/*********************************************************************************************************
** JOG
*********************************************************************************************************/

int SetJOGJointParams(JOGJointParams *jogJointParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (jogJointParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGJointParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], jogJointParams, sizeof(JOGJointParams));
    message.data()->paramsLen = sizeof(JOGJointParams);


    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetJOGJointParams(JOGJointParams *jogJointParams)
{
    // 0. Parameter checking
    if (jogJointParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGJointParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(jogJointParams, &message.data()->params[0], sizeof(JOGJointParams));

    return *result;
}

int SetJOGCoordinateParams(JOGCoordinateParams *jogCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (jogCoordinateParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGCoordinateParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], jogCoordinateParams, sizeof(JOGCoordinateParams));
    message.data()->paramsLen = sizeof(JOGCoordinateParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }
    return *result;
}

int GetJOGCoordinateParams(JOGCoordinateParams *jogCoordinateParams)
{
    // 0. Parameter checking
    if (jogCoordinateParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGCoordinateParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(jogCoordinateParams, &message.data()->params[0], sizeof(JOGCoordinateParams));

    return *result;
}

int SetJOGLParams(JOGLParams *jogLParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = false;
    // 0. Parameter checking
    if (jogLParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGLParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], jogLParams, sizeof(JOGLParams));
    message.data()->paramsLen = sizeof(JOGLParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }
    return *result;
}

int GetJOGLParams(JOGLParams *jogLParams)
{
    // 0. Parameter checking
    if (jogLParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGLParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(jogLParams, &message.data()->params[0], sizeof(ProtocolJOGLParams));

    return *result;
}

int SetJOGCommonParams(JOGCommonParams *jogCommonParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    // 0. Parameter checking
    if (jogCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGCommonParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], jogCommonParams, sizeof(JOGCommonParams));
    message.data()->paramsLen = sizeof(JOGCommonParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetJOGCommonParams(JOGCommonParams *jogCommonParams)
{
    // 0. Parameter checking
    if (jogCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolJOGCommonParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(jogCommonParams, &message.data()->params[0], sizeof(JOGCommonParams));

    return *result;
}

int SetJOGCmd(JOGCmd *jogCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = false;
    static bool isJointJog = false;

    // 0. Parameter checking
    if (jogCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    if (jogCmd->cmd != JogIdle) {
        isJointJog = jogCmd->isJoint;
    }
    message.data()->id = ProtocolJOGCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;

    message.data()->params[0] = isJointJog;
    message.data()->params[1] = jogCmd->cmd;
    message.data()->paramsLen = 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }
    return *result;
}

/*********************************************************************************************************
** PTP
*********************************************************************************************************/

int SetPTPJointParams(PTPJointParams *ptpJointParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpJointParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJointParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpJointParams, sizeof(PTPJointParams));
    message.data()->paramsLen = sizeof(PTPJointParams);


    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPJointParams(PTPJointParams *ptpJointParams)
{
    // 0. Parameter checking
    if (ptpJointParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJointParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpJointParams, &message.data()->params[0], sizeof(PTPJointParams));

    return *result;
}

int SetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpCoordinateParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPCoordinateParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpCoordinateParams, sizeof(PTPCoordinateParams));
    message.data()->paramsLen = sizeof(PTPCoordinateParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPCoordinateParams(PTPCoordinateParams *ptpCoordinateParams)
{
    // 0. Parameter checking
    if (ptpCoordinateParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPCoordinateParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpCoordinateParams, &message.data()->params[0], sizeof(PTPCoordinateParams));

    return *result;
}

int SetPTPLParams(PTPLParams *ptpLParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpLParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPLParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpLParams, sizeof(PTPLParams));
    message.data()->paramsLen = sizeof(PTPLParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPLParams(PTPLParams *ptpLParams)
{
    // 0. Parameter checking
    if (ptpLParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPLParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpLParams, &message.data()->params[0], sizeof(PTPLParams));

    return *result;
}

int SetPTPJumpParams(PTPJumpParams *ptpJumpParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpJumpParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJumpParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpJumpParams, sizeof(PTPJumpParams));
    message.data()->paramsLen = sizeof(PTPJumpParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPJumpParams(PTPJumpParams *ptpJumpParams)
{
    // 0. Parameter checking
    if (ptpJumpParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJumpParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpJumpParams, &message.data()->params[0], sizeof(PTPJumpParams));

    return *result;
}

int SetPTPJump2Params(PTPJump2Params *ptpJump2Params, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpJump2Params == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJump2Params;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpJump2Params, sizeof(PTPJump2Params));
    message.data()->paramsLen = sizeof(PTPJump2Params);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPJump2Params(PTPJump2Params *ptpJump2Params)
{
    // 0. Parameter checking
    if (ptpJump2Params == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPJump2Params;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpJump2Params, &message.data()->params[0], sizeof(PTPJump2Params));

    return *result;
}

int SetPTPCommonParams(PTPCommonParams *ptpCommonParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPCommonParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpCommonParams, sizeof(PTPCommonParams));
    message.data()->paramsLen = sizeof(PTPCommonParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetPTPCommonParams(PTPCommonParams *ptpCommonParams)
{
    // 0. Parameter checking
    if (ptpCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPCommonParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpCommonParams, &message.data()->params[0], sizeof(PTPCommonParams));

    return *result;
}

int SetPTPCmd(PTPCmd *ptpCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

   // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpCmd, sizeof(PTPCmd));
    message.data()->paramsLen = sizeof(PTPCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetPTPWithLCmd(PTPWithLCmd *ptpWithLCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpWithLCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

   // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPWithLCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpWithLCmd, sizeof(PTPWithLCmd));
    message.data()->paramsLen = sizeof(PTPWithLCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetPTPPOCmd(PTPCmd *ptpCmd, ParallelOutputCmd *parallelCmd, int parallelCmdCount, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

   // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPPOCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpCmd, sizeof(PTPCmd));
    if (parallelCmd && parallelCmdCount > 0) {
        message.data()->params[sizeof(PTPCmd)] = (uint8_t)parallelCmdCount;
        memcpy(&message.data()->params[sizeof(PTPCmd) + 1], parallelCmd, parallelCmdCount * sizeof(ParallelOutputCmd));
    } else {
        message.data()->params[sizeof(PTPCmd)] = 0;
    }
    message.data()->paramsLen = sizeof(PTPCmd) + 1 + parallelCmdCount * sizeof(ParallelOutputCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetPTPPOWithLCmd(PTPWithLCmd *ptpWithLCmd, ParallelOutputCmd *parallelCmd, int parallelCmdCount, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ptpWithLCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

   // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPPOWithLCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ptpWithLCmd, sizeof(PTPWithLCmd));
    if (parallelCmd && parallelCmdCount > 0) {
        message.data()->params[sizeof(PTPWithLCmd)] = (uint8_t)parallelCmdCount;
        memcpy(&message.data()->params[sizeof(PTPWithLCmd) + 1], parallelCmd, parallelCmdCount * sizeof(ParallelOutputCmd));
    } else {
        message.data()->params[sizeof(PTPWithLCmd)] = 0;
    }
    message.data()->paramsLen = sizeof(PTPWithLCmd) + 1 + parallelCmdCount * sizeof(ParallelOutputCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

/*********************************************************************************************************
** CP
*********************************************************************************************************/
int SetCPParams(CPParams *cpParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (cpParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], cpParams, sizeof(CPParams));
    message.data()->paramsLen = sizeof(CPParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetCPParams(CPParams *cpParams)
{
    // 0. Parameter checking
    if (cpParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(cpParams, &message.data()->params[0], sizeof(CPParams));

    return *result;
}

int SetCPCmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (cpCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], cpCmd, sizeof(CPCmd));
    message.data()->paramsLen = sizeof(CPCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetCPLECmd(CPCmd *cpCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (cpCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPLECmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], cpCmd, sizeof(CPCmd));
    message.data()->paramsLen = sizeof(CPCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetCPRHoldEnable(bool isEnable)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPRHoldEnable;
    message.data()->rw = 1;
    message.data()->isQueued = 0;
    message.data()->params[0] = (uint8_t)isEnable;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetCPRHoldEnable(bool *isEnable)
{
    // 0. Parameter checking
    if (isEnable == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPRHoldEnable;
    message.data()->rw = 0;
    message.data()->isQueued = 0;
    message.data()->params[0] = (uint8_t)*isEnable;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isEnable = (bool)message.data()->params[0];

    return *result;
}

/*********************************************************************************************************
** ARC
*********************************************************************************************************/

int SetARCParams(ARCParams *arcParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (arcParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolARCParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], arcParams, sizeof(ARCParams));
    message.data()->paramsLen = sizeof(ARCParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetARCParams(ARCParams *arcParams)
{
    // 0. Parameter checking
    if (arcParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolARCParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(arcParams, &message.data()->params[0], sizeof(ARCParams));

    return *result;
}

int SetARCCmd(ARCCmd *arcCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (arcCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolARCCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], arcCmd, sizeof(ARCCmd));
    message.data()->paramsLen = sizeof(ARCCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetCircleCmd(CircleCmd *circleCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (circleCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCircleCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], circleCmd, sizeof(CircleCmd));
    message.data()->paramsLen = sizeof(CircleCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

/*********************************************************************************************************
** WAIT
*********************************************************************************************************/

int SetWAITCmd(WAITCmd *waitCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (waitCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWAITCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], waitCmd, sizeof(WAITCmd));
    message.data()->paramsLen = sizeof(WAITCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

/*********************************************************************************************************
** TRIG
*********************************************************************************************************/

int SetTRIGCmd(TRIGCmd *trigCmd, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (trigCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolTRIGCmd;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], trigCmd, sizeof(TRIGCmd));
    message.data()->paramsLen = sizeof(TRIGCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

/*********************************************************************************************************
** EIO
*********************************************************************************************************/

int SetIOMultiplexing(IOMultiplexing *ioMultiplexing, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ioMultiplexing == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIOMultiplexing;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ioMultiplexing, sizeof(IOMultiplexing));
    message.data()->paramsLen = sizeof(IOMultiplexing);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetIOMultiplexing(IOMultiplexing *ioMultiplexing)
{
    // 0. Parameter checking
    if (ioMultiplexing == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIOMultiplexing;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ioMultiplexing, sizeof(IOMultiplexing));
    message.data()->paramsLen = sizeof(IOMultiplexing);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ioMultiplexing, &message.data()->params[0], sizeof(IOMultiplexing));

    return *result;
}

int SetIODO(IODO *ioDO, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ioDO == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIODO;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ioDO, sizeof(IODO));
    message.data()->paramsLen = sizeof(IODO);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetIODO(IODO *ioDO)
{
    // 0. Parameter checking
    if (ioDO == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIODO;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ioDO, sizeof(IODO));
    message.data()->paramsLen = sizeof(IODO);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ioDO, &message.data()->params[0], sizeof(IODO));

    return *result;
}

int SetIOPWM(IOPWM *ioPWM, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (ioPWM == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIOPWM;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], ioPWM, sizeof(IOPWM));
    message.data()->paramsLen = sizeof(IOPWM);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetIOPWM(IOPWM *ioPWM)
{
    // 0. Parameter checking
    if (ioPWM == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIOPWM;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ioPWM, sizeof(IOPWM));
    message.data()->paramsLen = sizeof(IOPWM);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ioPWM, &message.data()->params[0], sizeof(IOPWM));

    return *result;
}

int GetIODI(IODI *ioDI)
{
    // 0. Parameter checking
    if (ioDI == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIODI;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ioDI, sizeof(IODI));
    message.data()->paramsLen = sizeof(IODI);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ioDI, &message.data()->params[0], sizeof(IODI));

    return *result;
}

int GetIOADC(IOADC *ioADC)
{
    // 0. Parameter checking
    if (ioADC == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIOADC;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ioADC, sizeof(IOADC));
    message.data()->paramsLen = sizeof(IOADC);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ioADC, &message.data()->params[0], sizeof(IOADC));

    return *result;
}

int SetEMotor(EMotor *eMotor, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (eMotor == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEMotor;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], eMotor, sizeof(EMotor));
    message.data()->paramsLen = sizeof(EMotor);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetEMotorS(EMotorS *eMotorS, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (eMotorS == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolEMotorS;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], eMotorS, sizeof(EMotorS));
    message.data()->paramsLen = sizeof(EMotorS);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SetColorSensor(bool enable, ColorPort colorPort)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolColorSensor;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->params[0] = enable;
    message.data()->params[1] = (uint8_t)colorPort;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result

    return *result;
}

int GetColorSensor(uint8_t *r, uint8_t *g, uint8_t *b)
{
    // 0. Parameter checking
    if (r == NULL || g == NULL || b == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolColorSensor;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *r = message.data()->params[0];
    *g = message.data()->params[1];
    *b = message.data()->params[2];

    return *result;
}

int SetInfraredSensor(bool enable,InfraredPort infraredPort)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIRSwitch;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->params[0] = enable;
    message.data()->params[1] = (uint8_t)infraredPort;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result

    return *result;
}

int GetInfraredSensor(InfraredPort port, uint8_t *value)
{
    // 0. Parameter checking
    if (value== NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolIRSwitch;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->params[0] = (uint8_t)port;
    message.data()->params[1] = *value;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *value = message.data()->params[0];

    return *result;

}

/*********************************************************************************************************
** CAL
*********************************************************************************************************/

int SetAngleSensorStaticError(float rearArmAngleError, float frontArmAngleError)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAngleSensorStaticError;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy((void *)&message.data()->params[0], (void *)&rearArmAngleError, sizeof(float));
    memcpy((void *)&message.data()->params[sizeof(float)], (void *)&frontArmAngleError, sizeof(float));
    message.data()->paramsLen = 2 * sizeof(float);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetAngleSensorStaticError(float *rearArmAngleError, float *frontArmAngleError)
{
    // 0. Parameter checking
    if (rearArmAngleError == NULL ||
        frontArmAngleError == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAngleSensorStaticError;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy((void *)rearArmAngleError, &message.data()->params[0], sizeof(float));
    memcpy((void *)frontArmAngleError, &message.data()->params[sizeof(float)], sizeof(float));

    return *result;
}

int SetAngleSensorCoef(float rearArmAngleCoef, float frontArmAngleCoef)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAngleSensorCoef;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy((void *)&message.data()->params[0], (void *)&rearArmAngleCoef, sizeof(float));
    memcpy((void *)&message.data()->params[sizeof(float)], (void *)&frontArmAngleCoef, sizeof(float));
    message.data()->paramsLen = 2 * sizeof(float);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetAngleSensorCoef(float *rearArmAngleCoef, float *frontArmAngleCoef)
{
    // 0. Parameter checking
    if (rearArmAngleCoef == NULL ||
        frontArmAngleCoef == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolAngleSensorCoef;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy((void *)rearArmAngleCoef, &message.data()->params[0], sizeof(float));
    memcpy((void *)frontArmAngleCoef, &message.data()->params[sizeof(float)], sizeof(float));

    return *result;
}

int SetBaseDecoderStaticError(float baseDecoderError)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolBaseDecoderStaticError;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy((void *)&message.data()->params[0], (void *)&baseDecoderError, sizeof(float));
    message.data()->paramsLen = sizeof(float);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetBaseDecoderStaticError(float *baseDecoderError)
{
    // 0. Parameter checking
    if (baseDecoderError == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolBaseDecoderStaticError;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy((void *)baseDecoderError, &message.data()->params[0], sizeof(float));

    return *result;
}

int SetLRHandCalibrateValue(float lrHandCalibrateValue)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolLRHandCalibrateValue;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy((void *)&message.data()->params[0], (void *)&lrHandCalibrateValue, sizeof(float));
    message.data()->paramsLen = sizeof(float);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetLRHandCalibrateValue(float *lrHandCalibrateValue)
{
    // 0. Parameter checking
    if (lrHandCalibrateValue == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolLRHandCalibrateValue;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy((void *)lrHandCalibrateValue, &message.data()->params[0], sizeof(float));

    return *result;
}

/*********************************************************************************************************
** WIFI
*********************************************************************************************************/
int SetWIFIConfigMode(bool enable)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIConfigMode;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->params[0] = enable;
    message.data()->paramsLen = 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFIConfigMode(bool *isEnabled)
{
    // 0. Parameter checking
    if (isEnabled == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIConfigMode;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isEnabled = message.data()->params[0] != 0;
    return *result;
}

int SetWIFISSID(const char *ssid)
{
    // 0. Parameter checking
    if (ssid == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFISSID;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    strcpy((char *)&message.data()->params[0], ssid);
    message.data()->paramsLen = strlen(ssid) + 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFISSID(char *ssid, uint32_t maxLen)
{
    // 0. Parameter checking
    if (ssid == NULL ||
        maxLen == 0) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFISSID;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    uint32_t len = strlen((const char *)&message.data()->params[0]);
    if (len < maxLen) {
        strcpy(ssid, (const char *)&message.data()->params[0]);
    } else {
        memcpy(ssid, (const char *)&message.data()->params[0], maxLen - 1);
        ssid[maxLen - 1] = 0;
    }

    return *result;
}

int SetWIFIPassword(const char *password)
{
    // 0. Parameter checking
    if (password == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIPassword;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    strcpy((char *)&message.data()->params[0], password);
    message.data()->paramsLen = strlen(password) + 1;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFIPassword(char *password, uint32_t maxLen)
{
    // 0. Parameter checking
    if (password == NULL ||
        maxLen == 0) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIPassword;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    uint32_t len = strlen((const char *)&message.data()->params[0]);
    if (len < maxLen) {
        strcpy(password, (const char *)&message.data()->params[0]);
    } else {
        memcpy(password, (const char *)&message.data()->params[0], maxLen - 1);
        password[maxLen - 1] = 0;
    }

    return *result;
}

int SetWIFIIPAddress(WIFIIPAddress *wifiIPAddress)
{
    // 0. Parameter checking
    if (wifiIPAddress == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIIPAddress;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], wifiIPAddress, sizeof(WIFIIPAddress));
    message.data()->paramsLen = sizeof(WIFIIPAddress);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFIIPAddress(WIFIIPAddress *wifiIPAddress)
{
    // 0. Parameter checking
    if (wifiIPAddress == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIIPAddress;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(wifiIPAddress, &message.data()->params[0], sizeof(WIFIIPAddress));

    return *result;
}

int SetWIFINetmask(WIFINetmask *wifiNetmask)
{
    // 0. Parameter checking
    if (wifiNetmask == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFINetmask;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], wifiNetmask, sizeof(WIFINetmask));
    message.data()->paramsLen = sizeof(WIFINetmask);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFINetmask(WIFINetmask *wifiNetmask)
{
    // 0. Parameter checking
    if (wifiNetmask == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFINetmask;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(wifiNetmask, &message.data()->params[0], sizeof(WIFINetmask));

    return *result;
}

int SetWIFIGateway(WIFIGateway *wifiGateway)
{
    // 0. Parameter checking
    if (wifiGateway == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIGateway;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], wifiGateway, sizeof(WIFIGateway));
    message.data()->paramsLen = sizeof(WIFIGateway);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFIGateway(WIFIGateway *wifiGateway)
{
    // 0. Parameter checking
    if (wifiGateway == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIGateway;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(wifiGateway, &message.data()->params[0], sizeof(WIFIGateway));

    return *result;
}

int SetWIFIDNS(WIFIDNS *wifiDNS)
{
    // 0. Parameter checking
    if (wifiDNS == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIDNS;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], wifiDNS, sizeof(WIFIDNS));
    message.data()->paramsLen = sizeof(WIFIDNS);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetWIFIDNS(WIFIDNS *wifiDNS)
{
    // 0. Parameter checking
    if (wifiDNS == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIDNS;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(wifiDNS, &message.data()->params[0], sizeof(WIFIDNS));

    return *result;
}

int GetWIFIConnectStatus(bool *isConnected)
{
    // 0. Parameter checking
    if (isConnected == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolWIFIConnectStatus;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isConnected = message.data()->params[0];

    return *result;
}

int UpdateFirmware(FirmwareParams *firmwareParams)
{
    // 0. Parameter checking
    if (firmwareParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolFirmwareSwitch;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], firmwareParams, sizeof(FirmwareParams));
    message.data()->paramsLen = sizeof(FirmwareParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    return *result;
}

int SetFirmwareMode(FirmwareMode *firmwareMode)
{
    // 0. Parameter checking
    if (firmwareMode == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolFirmwareMode;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], firmwareMode, sizeof(FirmwareMode));
    message.data()->paramsLen = sizeof(FirmwareMode);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    return *result;
}

int GetFirmwareMode(FirmwareMode *firmwareMode)
{
    // 0. Parameter checking
    if (firmwareMode == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolFirmwareMode;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], firmwareMode, sizeof(FirmwareMode));
    message.data()->paramsLen = sizeof(FirmwareMode);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(firmwareMode, &message.data()->params[0], sizeof(FirmwareMode));

    return *result;
}

/*********************************************************************************************************
** Test
*********************************************************************************************************/
int GetUserParams(UserParams *userParams)
{
    // 0. Parameter checking
    if (userParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolUserParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(userParams, &message.data()->params[0], sizeof(UserParams));

    return *result;
}

int GetPTPTime(PTPCmd *ptpCmd, uint32_t *ptpTime)
{
    // 0. Parameter checking
    if (ptpCmd == NULL ||
        ptpTime == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolPTPTime;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], ptpCmd, sizeof(PTPCmd));
    message.data()->paramsLen = sizeof(PTPCmd);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(ptpTime, &message.data()->params[0], sizeof(uint32_t));

    return *result;
}

int GetServoPIDParams(PID *pid)
{
    // 0. Parameter checking
    if (pid == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolServoPIDParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], pid, sizeof(PID));
    message.data()->paramsLen = sizeof(PID);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(pid, &message.data()->params[0], sizeof(PID));

    qDebug() << "[DLL GetServoPID]"
             << " index:" << pid->index
             << ", controlLoop:" << pid->controlLoop
             << ", p:" << pid->params.p
             << ", i:" << pid->params.i
             << ", d:" << pid->params.d
             << ", v:" << pid->params.v
             << ", a:" << pid->params.a;
    return *result;
}

int SetServoPIDParams(PID *pid, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (pid == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolServoPIDParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], pid, sizeof(PID));
    message.data()->paramsLen = sizeof(PID);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    qDebug() << "[DLL SetServoPID]"
             << " index:" << pid->index
             << ", controlLoop:" << pid->controlLoop
             << ", p:" << pid->params.p
             << ", i:" << pid->params.i
             << ", d:" << pid->params.d
             << ", v:" << pid->params.v
             << ", a:" << pid->params.a;
    return *result;
}

int GetServoControlLoop(uint8_t index, uint8_t *controlLoop)
{
    // 0. Parameter checking
    if (controlLoop == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolServoControlLoop;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->params[0] = index;
    message.data()->params[1] = *controlLoop;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(controlLoop, &message.data()->params[0], sizeof(uint8_t));

    return *result;
}

int SetServoControlLoop(uint8_t index, uint8_t controlLoop, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolServoControlLoop;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = index;
    message.data()->params[1] = controlLoop;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SaveServoPIDParams(uint8_t index, uint8_t controlLoop, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolSaveServoPIDParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->params[0] = index;
    message.data()->params[1] = controlLoop;
    message.data()->paramsLen = sizeof(uint8_t) * 2;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

/*********************************************************************************************************
** Queud command
*********************************************************************************************************/

int SetQueuedCmdStartExec(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdStartExec;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetQueuedCmdStopExec(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdStopExec;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetQueuedCmdForceStopExec(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdForceStopExec;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetQueuedCmdStartDownload(uint32_t totalLoop, uint32_t linePerLoop)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdStartDownload;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    memcpy(&message.data()->params[0], &totalLoop, sizeof(uint32_t));
    memcpy(&message.data()->params[4], &linePerLoop, sizeof(uint32_t));
    message.data()->paramsLen = 2 * sizeof(uint32_t);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetQueuedCmdStopDownload(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdStopDownload;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetQueuedCmdClear(void)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdClear;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int GetQueuedCmdCurrentIndex(uint64_t *queuedCmdCurrentIndex)
{
    // 0. Parameter checking
    if (queuedCmdCurrentIndex == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolQueuedCmdCurrentIndex;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(queuedCmdCurrentIndex, &message.data()->params[0], sizeof(uint64_t));

    return *result;
}

int SetLostStepParams(float threshold)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolLostStepSet;
    message.data()->rw = 1;
    message.data()->isQueued = false;
    message.data()->paramsLen = sizeof(float);
    memcpy(&message.data()->params[0], &threshold, sizeof(float));

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    return *result;
}

int SetLostStepCmd(bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolLostStepDetect;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));

    return *result;

}

int GetUART4PeripheralsType(uint8_t *type)
{
    // 0. Parameter checking
    if(type == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCheckUART4PeripheralsModel;
    message.data()->rw = 0;
    message.data()->isQueued = 0;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *type = message.data()->params[0];

    return *result;
}

int SetUART4PeripheralsEnable(bool isEnable)
{
    // 0. Parameter checking

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolUART4PeripheralsEnabled;
    message.data()->rw = 1;
    message.data()->isQueued = 0;
    message.data()->paramsLen = 1;
    message.data()->params[0] = (uint8_t)isEnable;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result

    return *result;
}

int GetUART4PeripheralsEnable(bool *isEnable)
{
    // 0. Parameter checking
    if(isEnable == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolUART4PeripheralsEnabled;
    message.data()->rw = 0;
    message.data()->isQueued = 0;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    *isEnable = message.data()->params[0];

    return *result;
}

int SetCPCommonParams(CPCommonParams *cpCommonParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (cpCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPCommonParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], cpCommonParams, sizeof(CPCommonParams));
    message.data()->paramsLen = sizeof(CPCommonParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetCPCommonParams(CPCommonParams *cpCommonParams)
{
    // 0. Parameter checking
    if (cpCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolCPCommonParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(cpCommonParams, &message.data()->params[0], sizeof(CPCommonParams));

    return *result;
}

int SetARCCommonParams(ARCCommonParams *arcCommonParams, bool isQueued, uint64_t *queuedCmdIndex)
{
    isQueued = true;
    // 0. Parameter checking
    if (arcCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolARCCommonParams;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    memcpy(&message.data()->params[0], arcCommonParams, sizeof(ARCCommonParams));
    message.data()->paramsLen = sizeof(ARCCommonParams);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int GetARCCommonParams(ARCCommonParams *arcCommonParams)
{
    // 0. Parameter checking
    if (arcCommonParams == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolARCCommonParams;
    message.data()->rw = 0;
    message.data()->isQueued = false;
    message.data()->paramsLen = 0;

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    memcpy(arcCommonParams, &message.data()->params[0], sizeof(ARCCommonParams));

    return *result;
}

int SendPluse(PluseCmd *pluseCmd=NULL, bool isQueued=false, uint64_t *queuedCmdIndex=NULL)
{
    // 0. Parameter checking
    if(pluseCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }
    if (isQueued && queuedCmdIndex == NULL){
        return DobotCommunicate_InvalidParams;
    }

    // 1.Send the message
    QScopedPointer<Message> message(new Message);

    message.data()->id = ProtocolFunctionPulseMode;
    message.data()->rw = 1;
    message.data()->isQueued = isQueued;
    message.data()->paramsLen = sizeof(PluseCmd);
    memcpy(&message.data()->params[0], pluseCmd, message.data()->paramsLen);

    // 2.Wait for command execution
    WAIT_CMD_EXECUTION();

    // 3.The result
    if (isQueued && queuedCmdIndex) {
        memcpy(queuedCmdIndex, &message.data()->params[0], sizeof(uint64_t));
    }

    return *result;
}

int SendPluseEx(PluseCmd *pluseCmd=NULL)
{
    // 0. Parameter checking
    if(pluseCmd == NULL) {
        return DobotCommunicate_InvalidParams;
    }

    uint64_t index(0),curIndex(0);
    int res(0);

    res = SendPluse(pluseCmd, true, &index);
    while(!res){
        res = GetQueuedCmdCurrentIndex(&curIndex);
        if (res || curIndex >= index)
            break;
    }

    return res;
}
