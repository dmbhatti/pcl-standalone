#include "cloudemitterreceiver.h"

void CloudEmitterReceiver::connectToCloud(CloudEmitter *input) {
    connect(input, SIGNAL(newCloudMessage(const CloudMessage::ConstPtr)),
            this, SLOT(receiveCloudMessage(const CloudMessage::ConstPtr)));
}
