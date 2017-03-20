//! Declaration of the C++11 interface for the Velleman/OWI Robotic Arm.
/*!
 *  \file
 *  \author Maarten De Munck, <maarten@vijfendertig.be>
 *  \date 2017
 *  \copyright Licensed under the MIT License. See LICENSE for the full license.
 */


#ifndef __VIJFENDERTIG__ROBOTIC_ARM_USB__

  #define __VIJFENDERTIG__ROBOTIC_ARM_USB__


  #if __cplusplus < 201103L
    #error "The robotic arm interface requires at least a C++11 compliant compiler."
  #endif


  #include <condition_variable>
  #include <map>
  #include <mutex>
  #include <thread>
  #include <libusb-1.0/libusb.h>


  namespace vijfendertig {

    //! C++11 Interface to the Velleman/OWI robotic arm's USB interface.
    /*!
     *  This kit is known as:
     *  * Velleman Robotic Arm KSR10:
     *    http://www.velleman.eu/products/view/?id=375310
     *    http://www.velleman.eu/products/view/?id=379738
     *  * Owi Robotic Arm Edge or OWI-535:
     *    http://www.owirobot.com/robotic-arm-edge-1/
     *    http://www.owirobot.com/products/USB-Interface-for-Robotic-Arm-Edge.html
     */
    class RoboticArmUsb {

      public:

        //! Status and error definitions.
        enum class Status: int8_t {
          kDisconnected = 0,      //!< Disconnected from the robotic arm's USB interface.
          kConnecting = 1,        //!< Connecting to the robotic arm's USB interface.
          kConnected = 2,         //!< Connected to the robotic arm's USB interface.
          kIoError = 3,           //!< An I/O error occurred. Reconnecting is required.
          kDisconnecting = 4,     //!< Disconnecting from the robotic arm's USB interface.
          kDeviceNotFound = -1,   //!< The robotic arm's USB interface was not found.
          kConnectionFailed = -2, //!< The connection to the robotic arms's USB interface failed.
          kInvalidCommand = -3,   //!< The given command is not valid.
        };

        // Actuator definitions.
        enum class Actuator: uint8_t {
          kGripper = 0,  //!< Gripper (M1).
          kWrist = 2,    //!< Wrist (M2).
          kElbow = 4,    //!< Elbow (M3).
          kShoulder = 6, //!< Shoulder (M4).
          kBase = 8,     //!< Base (M5).
          kLight = 16    //!< Gripper light (LED).
        };

        //! Action definitions.
        enum class Action: uint8_t {
          kOff = 0,      //!< Turn off (light).
          kStop = 0,     //!< Stop (gripper, wrist, elbow, shoulder or base).
          kOn = 1,       //!< Turn on (light).
          kClose = 1,    //!< Close (gripper).
          kUp = 1,       //!< Move up (wrist, elbow or shoulder).
          kCW = 1,       //!< Move clockwise (base).
          kOpen = 2,     //!< Open (gripper).
          kDown = 2,     //!< Move down (wrist, elbow or shoulder).
          kCCW = 2       //!< Move counterclockwise (base).
        };

        RoboticArmUsb();
        RoboticArmUsb(const RoboticArmUsb &) = delete;
        virtual ~RoboticArmUsb();

        Status connect();
        Status disconnect();

        static bool isCommandValid(Actuator actuator, Action action);
        static bool isCommandValid(
            const std::map<RoboticArmUsb::Actuator, RoboticArmUsb::Action> & commands);
        Status sendCommand(Actuator actuator, Action action);
        Status sendCommand(
            const std::map<RoboticArmUsb::Actuator, RoboticArmUsb::Action> & commands);
        Status sendStop();

        Status getStatus() const;
        static std::string getStatusString(Status status);

      private:

        //! Default USB vendor ID.
        static const uint16_t default_vendor_id_{0x1267};
        //! Default USB product ID.
        static const uint16_t default_product_id_{0x0000};

        //! Raw command type.
        using Command = uint32_t;

        //! Mutex to serialise USB commands.
        mutable std::mutex serialise_mutex_;
        //! Condition variable to signal the initialisation's completion.
        std::condition_variable initialisation_finished_;
        //! Mutex for the condition variable to signal the initialisation's completion.
        std::mutex initialisation_finished_mutex_;
        //! Condition variable to signal a pending command to the control thread.
        std::condition_variable control_pending_;
        //! Mutex for the condition variable to signal a pending command to the control thread.
        std::mutex control_pending_mutex_;

        //! libusb context (to allow multiple libraries using libusb in the same application).
        libusb_context * libusb_context_;
        //! libusb device handle.
        libusb_device_handle * libusb_device_handle_;

        //! Current connection state.
        Status connection_state_;
        //! Current (raw) command state.
        Command command_state_;

        //! USB control thread.
        std::thread control_thread_;

        void controlThread();
        Status sendCommandState(Command command_state);
    };

  }

#endif // __VIJFENDERTIG__ROBOTIC_ARM_USB__
