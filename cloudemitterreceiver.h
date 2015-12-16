#ifndef CLOUDEMITTERRECEIVER_H
#define CLOUDEMITTERRECEIVER_H

#include <QObject>
#include "globals.h"

class CloudEmitter : public QObject {
    Q_OBJECT
signals:
    /** Emitted when a new cloud message is generated. */
    void newCloudMessage (const CloudMessage::ConstPtr &message);
};

class CloudEmitterReceiver : public CloudEmitter {
    Q_OBJECT
public:
    void connectToCloud(CloudEmitter *input);
public slots:
    /** Receives cloud messages. Pure virtual function, forces derived classes to implement it. */
    virtual void receiveCloudMessage(const CloudMessage::ConstPtr &message) = 0;
};

#endif // CLOUDEMITTERRECEIVER_H
