#ifndef VECTORNAV_BASE_IMPL_BASE_H
#define VECTORNAV_BASE_IMPL_BASE_H

#include <boost/thread.hpp>
#include <ossie/Device_impl.h>
#include <ossie/ThreadedComponent.h>

#include <frontend/frontend.h>
#include <ossie/MessageInterface.h>

class VectorNav_base : public Device_impl, protected ThreadedComponent
{
    public:
        VectorNav_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl);
        VectorNav_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev);
        VectorNav_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities);
        VectorNav_base(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev);
        ~VectorNav_base();

        void start() throw (CF::Resource::StartError, CORBA::SystemException);

        void stop() throw (CF::Resource::StopError, CORBA::SystemException);

        void releaseObject() throw (CF::LifeCycle::ReleaseError, CORBA::SystemException);

        void loadProperties();

    protected:
        // Member variables exposed as properties
        /// Property: device_kind
        std::string device_kind;
        /// Property: device_model
        std::string device_model;
        /// Property: SensorBaudrate
        CORBA::Long SensorBaudrate;
        /// Property: SensorPort
        std::string SensorPort;
        /// Property: use_gps_time
        bool use_gps_time;

        // Ports
        /// Port: dataNav_out
        frontend::OutNavDataPort *dataNav_out;
        /// Port: Navigation_out
        MessageSupplierPort *Navigation_out;

    private:
        void construct();
};
#endif // VECTORNAV_BASE_IMPL_BASE_H
