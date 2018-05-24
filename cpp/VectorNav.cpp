/**************************************************************************

    This is the device code. This file contains the child class where
    custom functionality can be added to the device. Custom
    functionality to the base class can be extended here. Access to
    the ports can also be done from this class

**************************************************************************/

#include "VectorNav.h"

PREPARE_LOGGING(VectorNav_i)

VectorNav_i::VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl) :
    VectorNav_base(devMgr_ior, id, lbl, sftwrPrfl)
{
}

VectorNav_i::VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, char *compDev) :
    VectorNav_base(devMgr_ior, id, lbl, sftwrPrfl, compDev)
{
}

VectorNav_i::VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities) :
    VectorNav_base(devMgr_ior, id, lbl, sftwrPrfl, capacities)
{
}

VectorNav_i::VectorNav_i(char *devMgr_ior, char *id, char *lbl, char *sftwrPrfl, CF::Properties capacities, char *compDev) :
    VectorNav_base(devMgr_ior, id, lbl, sftwrPrfl, capacities, compDev)
{
}

VectorNav_i::~VectorNav_i()
{
	vs.unregisterAsyncPacketReceivedHandler();
	vs.disconnect();
}

void VectorNav_i::constructor()
{
	// physical serial port  /dev/ttyS1
	// virtual (USB) serial port /dev/ttyUSB0

	// connect to VectorNav
	vs.connect(SensorPort, SensorBaudrate);

	// Query the sensor's model number.
	device_model = vs.readModelNumber();

	int async_output_rate;
	vs.writeAsyncDataOutputFrequency(async_output_rate);

	// Configure binary output message
	BinaryOutputRegister bor(
		ASYNCMODE_PORT1,
		1000 / async_output_rate,  // update rate [ms]
		COMMONGROUP_TIMESTARTUP | COMMONGROUP_TIMEGPS | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
		TIMEGROUP_NONE,
		IMUGROUP_NONE,
		GPSGROUP_NONE,
		ATTITUDEGROUP_NONE,
		INSGROUP_NONE);

	vs.writeBinaryOutput1(bor);
	vs.registerAsyncPacketReceivedHandler(NULL, asciiOrBinaryAsyncMessageReceived);

}


// Callback function to process data packet from sensor
void asciiOrBinaryAsyncMessageReceived(void* userData, Packet& p, size_t index)
{
	if (p.type() == Packet::TYPE_BINARY) {
		// First make sure we have a binary packet type we expect since there
		// are many types of binary output types that can be configured.
		if (!p.isCompatible(
			COMMONGROUP_TIMESTARTUP | COMMONGROUP_TIMEGPS | COMMONGROUP_YAWPITCHROLL | COMMONGROUP_QUATERNION | COMMONGROUP_ANGULARRATE | COMMONGROUP_POSITION | COMMONGROUP_VELOCITY | COMMONGROUP_ACCEL | COMMONGROUP_MAGPRES,
			TIMEGROUP_NONE,
			IMUGROUP_NONE,
			GPSGROUP_NONE,
			ATTITUDEGROUP_NONE,
			INSGROUP_NONE)) {
			// Not the type of binary packet we are expecting.
			return;
		}

		// re-format to Redhawk Nav message
		frontend::NavigationPacket navdata;

		// COMMONGROUP_TIMESTARTUP
		uint64_t timeStartup = p.extractUint64();

		// COMMONGROUP_TIMEGPS
		uint64_t timeGPS = p.extractUint64();

		// TODO need this function as part of class
		//if (this->use_gps_time) {
			// TODO match time format
			//navdata.timestamp = timeGPS;
		//} else {
			navdata.timestamp = bulkio::time::utils::now();
		//}

		// COMMONGROUP_YAWPITCHROLL
		vec3f ypr = p.extractVec3f();
		navdata.attitude.pitch = ypr[0];
		navdata.attitude.roll = ypr[1];
		navdata.attitude.yaw = ypr[2];

		// COMMONGROUP_QUATERNION
		//TODO add quaternion info to Nav IDL
		vec4f q = p.extractVec4f();
		navdata.cposition.x = q[0];
		navdata.cposition.y = q[1];
		navdata.cposition.z = q[2];
		//msgIMU.orientation.w = q[3];

		// COMMONGROUP_ANGULARRATE
		vec3f ar = p.extractVec3f();
		//TODO add angular rate info to Nav IDL
		//navdata.angular_rate.x = ar[0];
		//navdata.angular_rate.y = ar[1];
		//navdata.angular_rate.z = ar[2];

		// COMMONGROUP_POSITION
		vec3d lla = p.extractVec3d();
		navdata.position.lat = lla[0];
		navdata.position.lon = lla[1];
		navdata.position.alt = lla[2];

		// COMMONGROUP_VELOCITY
		vec3f vel = p.extractVec3f();
		navdata.velocity.x = vel[0];
		navdata.velocity.y = vel[1];
		navdata.velocity.z = vel[2];

		// COMMONGROUP_ACCEL
		vec3f al = p.extractVec3f();
		navdata.acceleration.x = al[0];
		navdata.acceleration.y = al[1];
		navdata.acceleration.z = al[2];

		// COMMONGROUP_MAGPRES
		//TODO add magnetic field info to Nav IDL
		vec3f mag = p.extractVec3f();
		//navdata.magnetic_field.x = mag[0];
		//navdata.magnetic_field.y = mag[1];
		//navdata.magnetic_field.z = mag[2];

		// send Redhawk packet
		// TODO is this how navigation ports work? Do we need to overload the get_nav_packet() function?
		frontend::returnNavigationPacket(navdata);
		// TODO do this instead? if active/connected then send packet? need to be in class...
		//if (this->dataNav_out->isActive()) {
		//	this->dataNav_out->nav_packet(navdata);
		//}

		// TODO how you do event ports, but need this function as part of class
		//this->Navigation_out->sendMessage(navdata);
	}
}


/**************************************************************************

    This is called automatically after allocateCapacity or deallocateCapacity are called.
    Your implementation should determine the current state of the device:

       setUsageState(CF::Device::IDLE);   // not in use
       setUsageState(CF::Device::ACTIVE); // in use, with capacity remaining for allocation
       setUsageState(CF::Device::BUSY);   // in use, with no capacity remaining for allocation

**************************************************************************/
void VectorNav_i::updateUsageState()
{
}

/***********************************************************************************************

    Basic functionality:

        The service function is called by the serviceThread object (of type ProcessThread).
        This call happens immediately after the previous call if the return value for
        the previous call was NORMAL.
        If the return value for the previous call was NOOP, then the serviceThread waits
        an amount of time defined in the serviceThread's constructor.
        
    SRI:
        To create a StreamSRI object, use the following code:
                std::string stream_id = "testStream";
                BULKIO::StreamSRI sri = bulkio::sri::create(stream_id);

    Time:
        To create a PrecisionUTCTime object, use the following code:
                BULKIO::PrecisionUTCTime tstamp = bulkio::time::utils::now();

        
    Ports:

        Data is passed to the serviceFunction through by reading from input streams
        (BulkIO only). The input stream class is a port-specific class, so each port
        implementing the BulkIO interface will have its own type-specific input stream.
        UDP multicast (dataSDDS and dataVITA49) ports do not support streams.

        The input stream from which to read can be requested with the getCurrentStream()
        method. The optional argument to getCurrentStream() is a floating point number that
        specifies the time to wait in seconds. A zero value is non-blocking. A negative value
        is blocking.  Constants have been defined for these values, bulkio::Const::BLOCKING and
        bulkio::Const::NON_BLOCKING.

        More advanced uses of input streams are possible; refer to the REDHAWK documentation
        for more details.

        Input streams return data blocks that automatically manage the memory for the data
        and include the SRI that was in effect at the time the data was received. It is not
        necessary to delete the block; it will be cleaned up when it goes out of scope.

        To send data using a BulkIO interface, create an output stream and write the
        data to it. When done with the output stream, the close() method sends and end-of-
        stream flag and cleans up.

        NOTE: If you have a BULKIO dataSDDS or dataVITA49  port, you must manually call 
              "port->updateStats()" to update the port statistics when appropriate.

        Example:
            // This example assumes that the device has two ports:
            //  An input (provides) port of type bulkio::InShortPort called dataShort_in
            //  An output (uses) port of type bulkio::OutFloatPort called dataFloat_out
            // The mapping between the port and the class is found
            // in the device base class header file

            bulkio::InShortStream inputStream = dataShort_in->getCurrentStream();
            if (!inputStream) { // No streams are available
                return NOOP;
            }

            // Get the output stream, creating it if it doesn't exist yet
            bulkio::OutFloatStream outputStream = dataFloat_out->getStream(inputStream.streamID());
            if (!outputStream) {
                outputStream = dataFloat_out->createStream(inputStream.sri());
            }

            bulkio::ShortDataBlock block = inputStream.read();
            if (!block) { // No data available
                // Propagate end-of-stream
                if (inputStream.eos()) {
                   outputStream.close();
                }
                return NOOP;
            }

            if (block.sriChanged()) {
                // Update output SRI
                outputStream.sri(block.sri());
            }

            // Get read-only access to the input data
            redhawk::shared_buffer<short> inputData = block.buffer();

            // Acquire a new buffer to hold the output data
            redhawk::buffer<float> outputData(inputData.size());

            // Transform input data into output data
            for (size_t index = 0; index < inputData.size(); ++index) {
                outputData[index] = (float) inputData[index];
            }

            // Write to the output stream; outputData must not be modified after
            // this method call
            outputStream.write(outputData, block.getStartTime());

            return NORMAL;

        If working with complex data (i.e., the "mode" on the SRI is set to
        true), the data block's complex() method will return true. Data blocks
        provide a cxbuffer() method that returns a complex interpretation of the
        buffer without making a copy:

            if (block.complex()) {
                redhawk::shared_buffer<std::complex<short> > inData = block.cxbuffer();
                redhawk::buffer<std::complex<float> > outData(inData.size());
                for (size_t index = 0; index < inData.size(); ++index) {
                    outData[index] = inData[index];
                }
                outputStream.write(outData, block.getStartTime());
            }

        Interactions with non-BULKIO ports are left up to the device developer's discretion
        
    Messages:
    
        To receive a message, you need (1) an input port of type MessageEvent, (2) a message prototype described
        as a structure property of kind message, (3) a callback to service the message, and (4) to register the callback
        with the input port.
        
        Assuming a property of type message is declared called "my_msg", an input port called "msg_input" is declared of
        type MessageEvent, create the following code:
        
        void VectorNav_i::my_message_callback(const std::string& id, const my_msg_struct &msg){
        }
        
        Register the message callback onto the input port with the following form:
        this->msg_input->registerMessage("my_msg", this, &VectorNav_i::my_message_callback);
        
        To send a message, you need to (1) create a message structure, (2) a message prototype described
        as a structure property of kind message, and (3) send the message over the port.
        
        Assuming a property of type message is declared called "my_msg", an output port called "msg_output" is declared of
        type MessageEvent, create the following code:
        
        ::my_msg_struct msg_out;
        this->msg_output->sendMessage(msg_out);

    Accessing the Device Manager and Domain Manager:
    
        Both the Device Manager hosting this Device and the Domain Manager hosting
        the Device Manager are available to the Device.
        
        To access the Domain Manager:
            CF::DomainManager_ptr dommgr = this->getDomainManager()->getRef();
        To access the Device Manager:
            CF::DeviceManager_ptr devmgr = this->getDeviceManager()->getRef();
    
    Properties:
        
        Properties are accessed directly as member variables. For example, if the
        property name is "baudRate", it may be accessed within member functions as
        "baudRate". Unnamed properties are given the property id as its name.
        Property types are mapped to the nearest C++ type, (e.g. "string" becomes
        "std::string"). All generated properties are declared in the base class
        (VectorNav_base).
    
        Simple sequence properties are mapped to "std::vector" of the simple type.
        Struct properties, if used, are mapped to C++ structs defined in the
        generated file "struct_props.h". Field names are taken from the name in
        the properties file; if no name is given, a generated name of the form
        "field_n" is used, where "n" is the ordinal number of the field.
        
        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A boolean called scaleInput
              
            if (scaleInput) {
                dataOut[i] = dataIn[i] * scaleValue;
            } else {
                dataOut[i] = dataIn[i];
            }
            
        Callback methods can be associated with a property so that the methods are
        called each time the property value changes.  This is done by calling 
        addPropertyListener(<property>, this, &VectorNav_i::<callback method>)
        in the constructor.

        The callback method receives two arguments, the old and new values, and
        should return nothing (void). The arguments can be passed by value,
        receiving a copy (preferred for primitive types), or by const reference
        (preferred for strings, structs and vectors).

        Example:
            // This example makes use of the following Properties:
            //  - A float value called scaleValue
            //  - A struct property called status
            
        //Add to VectorNav.cpp
        VectorNav_i::VectorNav_i(const char *uuid, const char *label) :
            VectorNav_base(uuid, label)
        {
            addPropertyListener(scaleValue, this, &VectorNav_i::scaleChanged);
            addPropertyListener(status, this, &VectorNav_i::statusChanged);
        }

        void VectorNav_i::scaleChanged(float oldValue, float newValue)
        {
            LOG_DEBUG(VectorNav_i, "scaleValue changed from" << oldValue << " to " << newValue);
        }
            
        void VectorNav_i::statusChanged(const status_struct& oldValue, const status_struct& newValue)
        {
            LOG_DEBUG(VectorNav_i, "status changed");
        }
            
        //Add to VectorNav.h
        void scaleChanged(float oldValue, float newValue);
        void statusChanged(const status_struct& oldValue, const status_struct& newValue);
        
    Allocation:
    
        Allocation callbacks are available to customize the Device's response to 
        allocation requests. For example, if the Device contains the allocation 
        property "my_alloc" of type string, the allocation and deallocation
        callbacks follow the pattern (with arbitrary function names
        my_alloc_fn and my_dealloc_fn):
        
        bool VectorNav_i::my_alloc_fn(const std::string &value)
        {
            // perform logic
            return true; // successful allocation
        }
        void VectorNav_i::my_dealloc_fn(const std::string &value)
        {
            // perform logic
        }
        
        The allocation and deallocation functions are then registered with the Device
        base class with the setAllocationImpl call. Note that the variable for the property is used rather
        than its id:
        
        this->setAllocationImpl(my_alloc, this, &VectorNav_i::my_alloc_fn, &VectorNav_i::my_dealloc_fn);
        
        

************************************************************************************************/
int VectorNav_i::serviceFunction()
{
    LOG_DEBUG(VectorNav_i, "serviceFunction() example log message");
    
    return NOOP;
}

