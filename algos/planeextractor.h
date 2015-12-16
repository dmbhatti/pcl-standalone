#ifndef PLANEEXTRACTOR_H
#define PLANEEXTRACTOR_H
#include <QFuture>

#include "../cloudemitterreceiver.h"

class PlaneExtractor : public CloudEmitterReceiver
{
    Q_OBJECT

public:
    PlaneExtractor();

public slots:
    void receiveCloudMessage(const CloudMessage::ConstPtr &message);

private:
    void algorithm(const CloudMessage::ConstPtr &message);
    void dummy();
    int instanceID;

    QFuture<void> thread1;
    QFuture<void> thread2;
};

#endif // PLANEEXTRACTOR_H
