#ifndef VECTORNAV_I_IMPL_H
#define VECTORNAV_I_IMPL_H

#include "VectorNav_base.h"
#include <iostream>
#include "vn/sensors.h"  // VectorNav sensors

using namespace std;
using namespace vn::math;
using namespace vn::sensors;
using namespace vn::protocol::uart;
using namespace vn::xplat;

// TODO if this is declared in the class, you get an <unresolved overloaded function type> error....
// but I need it as part of class so I can call ports from within!
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index);

class VectorNav_i : public VectorNav_base
{
    ENABLE_LOGGING
    public:
        VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~VectorNav_i();

        void constructor();

        int serviceFunction();

    	// Create a VnSensor object
    	VnSensor vs;

    protected:
        void updateUsageState();
};

#endif // VECTORNAV_I_IMPL_H
