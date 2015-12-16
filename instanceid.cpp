#include "instanceid.h"

// Initialize counter
int InstanceID::counter(0);

int InstanceID::getUniqueID() {
    return counter++;
}
