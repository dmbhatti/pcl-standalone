#ifndef INSTANCEID_H
#define INSTANCEID_H

class InstanceID {
public:
    static int getUniqueID();
private:
    static int counter;
};


#endif // INSTANCEID_H
