#ifndef __VIJFENDERTIG__ROBOTIC_ARM_USB__

  #define __VIJFENDERTIG__ROBOTIC_ARM_USB__


  #if __cplusplus < 201103L
    #error "The robotic arm interface requires at least a C++11 compliant compiler."
  #endif


  #include <atomic>
  #include <condition_variable>
  #include <map>
  #include <mutex>
  #include <thread>
  #include <libusb-1.0/libusb.h>


  namespace vijfendertig {

    //! Interface to the Velleman/OWI robotic arm's USB interface.
    /*!
     *  This kit is known as:
     *  * Velleman Robotic Arm KSR10:
     *    http://www.velleman.eu/products/view/?id=375310
     *    http://www.velleman.eu/products/view/?id=379738
     *  * Owi Robotic Arm Edge or OWI-535:
     *    http://www.owirobot.com/robotic-arm-edge-1/
     *    http://www.owirobot.com/products/USB-Interface-for-Robotic-Arm-Edge.html
     *
     *  \author Maarten De Munck <maarten@vijfendertig.be>
     */
    class RoboticArmUsb {

      public:

        enum class Status: int8_t {
          kDisconnected = 0,
          kConnecting = 1,
          kConnected = 2,
          kIoError = 3,
          kDisconnecting = 4,
          kDeviceNotFound = -1,
          kConnectionFailed = -2,
          kInvalidCommand = -3,
        };

        enum class Actuator: uint8_t {
          kGripper = 0,
          kWrist = 2,
          kElbow = 4,
          kShoulder = 6,
          kBase = 8,
          kLight = 16
        };

        enum class Action: uint8_t {
          kOff = 0,
          kStop = 0,
          kOn = 1,
          kClose = 1,
          kUp = 1,
          kCW = 1,
          kOpen = 2,
          kDown = 2,
          kCCW = 2
        };

        RoboticArmUsb();
        RoboticArmUsb(const RoboticArmUsb &) = delete;
        virtual ~RoboticArmUsb();

        Status connect();
        Status disconnect();

        static bool isCommandValid(Actuator actuator, Action action);
        Status sendCommand(Actuator actuator, Action action);
        Status sendCommand(const std::map<Actuator, Action> & commands);
        Status sendStop();

        Status getStatus() const;
        static std::string getStatusString(Status status);

      private:

        static const uint16_t default_vendor_id_{0x1267};
        static const uint16_t default_product_id_{0x0000};

        using Command = uint32_t;

        mutable std::mutex serialise_mutex_;
        std::condition_variable initialisation_finished_;
        std::mutex initialisation_finished_mutex_;
        std::condition_variable control_pending_;
        std::mutex control_pending_mutex_;

        libusb_context * libusb_context_;
        libusb_device_handle * libusb_device_handle_;

        Status connection_state_;
        Command command_state_;

        std::thread control_thread_;

        void controlThread();
        Status sendCommandState(Command command_state);
    };

  }

#endif // __VIJFENDERTIG__ROBOTIC_ARM_USB__
