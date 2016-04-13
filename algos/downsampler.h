#ifndef DOWNSAMPLER_H
#define DOWNSAMPLER_H

#include <QVector>
#include <QFuture>
#include "../cloudemitterreceiver.h"

class Downsampler : public CloudEmitterReceiver
{
Q_OBJECT

public:
  Downsampler();

public slots:
  void receiveCloudMessage(const CloudMessage::ConstPtr &message);

private:
  void algorithm(const CloudMessage::ConstPtr &message);
  void dummy();
  int instanceID;

  QVector<QFuture<void> > threads;
};

#endif // DOWNSAMPLER_H
