//! Implementation of the C++11 interface for the Velleman/OWI Robotic Arm.
/*!
 *  \file
 *  \author Maarten De Munck, <maarten@vijfendertig.be>
 *  \date 2017
 *  \copyright Licensed under the MIT License. See LICENSE for the full license.
 */


#if __cplusplus < 201103L
  #error "The robotic arm interface requires at least a C++11 compliant compiler."
#endif


#include <robotic-arm-usb.h>

#include <algorithm>
#include <iostream>


namespace vijfendertig {

  //! Create a new robotic arm controller object.
  /*!
   *  This function initialises the robotic arm controller object and the libusb library. It will
   *  not connect to or even claim the robotic arm's USB device (use the connect() function for
   *  that).
   */
  RoboticArmUsb::RoboticArmUsb():
    libusb_context_{nullptr},
    libusb_device_handle_{nullptr},
    connection_state_{Status::kDisconnected},
    command_state_{0}
  {
    // Initialise libusb.
    if(int error = libusb_init(&libusb_context_) != LIBUSB_SUCCESS) {
      std::string message{"An error occured while initialising the robotic arm driver: "
        "libusb error: " + std::string(libusb_error_name(error))
        + " (" + std::to_string(error) + ")"};
      std::cerr << message << "." << std::endl;
      throw std::runtime_error(message);
    }
  }

  //! Destroy a robotic arm controller object.
  /*!
   *  This function will stop the robotic arm's actuators and disconnect from the robotic arm's
   *  USB device (if not yet done) and deinitialise the libusb library.
   */
  RoboticArmUsb::~RoboticArmUsb()
  {
    // Disconnect robotic arm.
    disconnect();
    // Deinitialize libusb.
    if(libusb_context_ != nullptr) {
      libusb_exit(libusb_context_);
    }
  }

  //! Connect to a robotic arm's USB device.
  /*!
   *  This function will look for (the first available) robotic arm's USB interface, connect to it
   *  and claim it. It will start separate control thread which will perform all communication to
   *  the USB device. If the object is already connected to a USB device, a second call to this
   *  function will be ignored.
   *
   *  \returns kConnected on success, kDeviceNotFound or kConnectionFailed on failure.
   */
  RoboticArmUsb::Status RoboticArmUsb::connect()
  {
    std::lock_guard<std::mutex> lock{serialise_mutex_};
    Status connection_state_return{Status::kConnected}; 
    if(libusb_device_handle_ == nullptr) {
      if(connection_state_ != Status::kDisconnected) {
        std::string message{"Assertion failed: "
          "libusb_device_handle_ == nullptr && connection_state_ != kDisconnected"};
        std::cerr << message << "." << std::endl;
        throw std::logic_error(message);
      }
      connection_state_ = Status::kConnecting;
      libusb_device ** device_list{nullptr};
      libusb_device * device_found{nullptr};
      ssize_t device_count = libusb_get_device_list(libusb_context_, &device_list);
      for(ssize_t device_iterator = 0; device_iterator < device_count; ++ device_iterator) {
        libusb_device_descriptor device_descriptor;
        if(libusb_get_device_descriptor(device_list[device_iterator], &device_descriptor) == 0) {
          if(device_descriptor.idVendor == default_vendor_id_ &&
              device_descriptor.idProduct == default_product_id_) {
            device_found = device_list[device_iterator];
            break;
          }
        }
      }
      if(device_found != nullptr) {
        if(int error = libusb_open(device_found, &libusb_device_handle_) != LIBUSB_SUCCESS) {
          std::string message{"An error occured while opening the robotic arm's USB device: "
            "libusb error: " + std::string(libusb_error_name(error))
            + " (" + std::to_string(error) + ")"};
          std::cerr << message << "." << std::endl;
          connection_state_ = Status::kDisconnected;
          connection_state_return = Status::kConnectionFailed;
        }
        else {
          if(int error = libusb_claim_interface(libusb_device_handle_, 0) != LIBUSB_SUCCESS) {
            std::string message{"An error occured while claiming the robotic arm's USB interface: "
              "libusb error: " + std::string(libusb_error_name(error))
              + " (" + std::to_string(error) + ")"};
            std::cerr << message << "." << std::endl;
            connection_state_ = Status::kDisconnected;
            connection_state_return = Status::kConnectionFailed;
          }
          else {
            control_thread_ = std::thread(&RoboticArmUsb::controlThread, this);
            std::unique_lock<std::mutex> initialisation_lock{initialisation_finished_mutex_};
            initialisation_finished_.wait(initialisation_lock,
                [this]{return connection_state_ != Status::kConnecting;});
            connection_state_return = connection_state_;
          }
        }
      }
      else {
        std::string message{"The robotic arm's USB device is not found"};
        std::cerr << message << "." << std::endl;
        connection_state_ = Status::kDisconnected;
        connection_state_return = Status::kDeviceNotFound;
      }
      libusb_free_device_list(device_list, 1);
    }
    return connection_state_return;
  }

  //! Disconnect from the robotic arm's USB device.
  /*!
   *  This function will stop the robotic arm, terminate (and join) the separate control thread,
   *  disconnect from the robotic arm's USB interface and release it. If the object is not
   *  connected to a USB device, a call to this function will be ignored.
   *
   *  \return kDisconnected.
   */
  RoboticArmUsb::Status RoboticArmUsb::disconnect()
  {
    std::lock_guard<std::mutex> lock{serialise_mutex_};
    if(libusb_device_handle_ != nullptr) {
      if(connection_state_ == Status::kDisconnected) {
        std::string message{"Assertion failed: "
          "libusb_device_handle_ != nullptr && connection_state_ == kDisconnected"};
        std::cerr << message << "." << std::endl;
        throw std::logic_error(message);
      }
      {
        std::lock_guard<std::mutex> lock{control_pending_mutex_};
        connection_state_ = Status::kDisconnecting;
        control_pending_.notify_all();
      }
      if(control_thread_.joinable()) {
        control_thread_.join();
      }
      else {
        std::string message{"Assertion failed: control_thread_.joinable() != true"};
        std::cerr << message << "." << std::endl;
        throw std::logic_error(message);
      }
      libusb_release_interface(libusb_device_handle_, 0);
      libusb_close(libusb_device_handle_);
      libusb_device_handle_ = nullptr;
    }
    connection_state_ = Status::kDisconnected;
    return Status::kDisconnected;
  }

  //! Verify whether a given command is valid.
  /*!
   *  \param actuator Actuator.
   *  \param action Action.
   *  \return True if the given action is valid for the actuator, false if not.
   */
  bool RoboticArmUsb::isCommandValid(
      RoboticArmUsb::Actuator actuator, RoboticArmUsb::Action action)
  {
    if(((actuator == Actuator::kGripper)
          && (action == Action::kStop || action == Action::kClose || action == Action::kOpen))
        || ((actuator == Actuator::kWrist || actuator == Actuator::kElbow
            || actuator == Actuator::kShoulder)
          && (action == Action::kStop || action == Action::kUp || action == Action::kDown))
        || ((actuator == Actuator::kBase)
          && (action == Action::kStop || action == Action::kCW || action == Action::kCCW))
        || ((actuator == Actuator::kLight)
          && (action == Action::kOn || action == Action::kOff))) {
      return true;
    }
    else {
      return false;
    }
  }

  //! Verify whether a given composite command is valid.
  /*!
   *  \param commands Composite (actuator/action) command.
   *  \return True if the given actions are valid for their respective actuators, false if not.
   */
  bool RoboticArmUsb::isCommandValid(
      const std::map<RoboticArmUsb::Actuator, RoboticArmUsb::Action> & commands)
  {
    if(std::all_of(
          commands.begin(), commands.end(),
          [](const std::pair<Actuator, Action> & command) {
            return isCommandValid(command.first, command.second);
          })) {
      return true;
    }
    else {
      return false;
    }
  }

  //! Send a command to the robotic arm's interface.
  /*!
   *  \param actuator Actuator.
   *  \param action Action.
   *  \return kConnected on success, kInvalidCommand if the given command was not valid or
   *      kIoError on USB errors.
   */
  RoboticArmUsb::Status RoboticArmUsb::sendCommand(
      RoboticArmUsb::Actuator actuator, RoboticArmUsb::Action action)
  {
    // Check whether we received a valid command before aqcuiring the mutex.
    if(!isCommandValid(actuator, action)) {
      return Status::kInvalidCommand;
    }
    else {
      std::lock_guard<std::mutex> lock{serialise_mutex_};
      if(connection_state_ == Status::kConnected) {
        // Get lock, update command state and notify control thread.
        std::lock_guard<std::mutex> lock{control_pending_mutex_};
        command_state_ &= ~(0x03 << uint8_t(actuator));
        command_state_ |= uint8_t(action) << uint8_t(actuator);
        control_pending_.notify_all();
      }
      return connection_state_;
    }
  }

  //! Send a composite command to the robotic arm's interface.
  /*!
   *  \param commands Composite (actuator/action) command.
   *  \return kConnected on success, kInvalidCommand if at least one of the given commands was not
   *      valid or kIoError on USB errors.
   */
  RoboticArmUsb::Status RoboticArmUsb::sendCommand(
      const std::map<RoboticArmUsb::Actuator, RoboticArmUsb::Action> & commands)
  {
    // Check whether we received a valid command before aqcuiring the mutex.
    if(!isCommandValid(commands)) {
      return Status::kInvalidCommand;
    }
    else {
      std::lock_guard<std::mutex> lock{serialise_mutex_};
      if(connection_state_ == Status::kConnected) {
        // Get lock, update command state and notify control thread.
        std::lock_guard<std::mutex> lock{control_pending_mutex_};
        for(const auto & command: commands) {
          command_state_ &= ~(0x03 << uint8_t(command.first));
          command_state_ |= uint8_t(command.second) << uint8_t(command.first);
        }
        control_pending_.notify_all();
      }
      return connection_state_;
    }
  }

  //! Send a stop command to the robotic arm's interface.
  /*!
   *  \return kConnected on success or kIoError on USB errors.
   */
  RoboticArmUsb::Status RoboticArmUsb::sendStop()
  {
    Command command_state_stop = 0;
    std::lock_guard<std::mutex> lock{serialise_mutex_};
    if(connection_state_ == Status::kConnected && command_state_ != command_state_stop) {
      // Get lock, update command state and notify control thread.
      std::lock_guard<std::mutex> lock{control_pending_mutex_};
      command_state_ = command_state_stop;
      control_pending_.notify_all();
    }
    return connection_state_;
  }

  //! Get the current status of the robotic arm's control object.
  /*!
   *  \return kDisconnected, kConnecting, kConnected, kIoError or kDisconnected, depending on the
   *      current state of the robotic arm's control object.
   */
  RoboticArmUsb::Status RoboticArmUsb::getStatus() const
  {
    std::lock_guard<std::mutex> lock{serialise_mutex_};
    return connection_state_;
  }

  //! Translate a status code to a human readable status string.
  /*!
   *  \param status Status code to translate.
   *  \return All lowercase string representation of the given status.
   */
  std::string RoboticArmUsb::getStatusString(RoboticArmUsb::Status status)
  {
    switch(status) {
      case Status::kDisconnected: return "disconnected";
      case Status::kConnecting: return "connecting";
      case Status::kConnected: return "connected";
      case Status::kIoError: return "input/output error";
      case Status::kDisconnecting: return "disconnecting";
      case Status::kDeviceNotFound: return "device not found";
      case Status::kConnectionFailed: return "connection failed";
      case Status::kInvalidCommand: return "invalid command";
      default: return "other error";
    }
  }

  //! Control thread.
  void RoboticArmUsb::controlThread()
  {
    Status connection_state_current{Status::kConnecting};
    Command command_state_current{0};
    // Stop device prior to entering the control loop.
    { // lock_guard scope.
      std::lock_guard<std::mutex> lock{initialisation_finished_mutex_};
      command_state_ = 0;
      connection_state_ = sendCommandState(command_state_);
      connection_state_current = connection_state_;
      command_state_current = command_state_;
      initialisation_finished_.notify_all();
    }
    // Control loop. Process new commands as they are generated by other threads.
    do {
      std::unique_lock<std::mutex> lock(control_pending_mutex_);
      control_pending_.wait(lock,
          [this, connection_state_current, command_state_current]
          {return connection_state_current != connection_state_
                  || command_state_current != command_state_;});
      if(command_state_ != command_state_current && connection_state_ == Status::kConnected) {
        connection_state_ = sendCommandState(command_state_);
      }
      command_state_current = command_state_;
      connection_state_current = connection_state_;
    } while(connection_state_current == Status::kConnected);
    // Stop device prior to disconnecting.
    // The connect() function only touches libusb before starting this thread, the disconnect()
    // function only touches libusb after starting this thread and all setCommandState() calls
    // should be done by this thread, so it's safe to reset the device without locking a mutex.
    sendCommandState(0);
  }

  //! Send a raw command to the robotic arm's USB interface.
  /*!
   *  Based on the "OWI Robotic Arm Edge USB protocol (and sampe code)" article at
   *  <http://notbrainsurgery.livejournal.com/38622.html> by Vadim Zaliva
   *  <http://www.crocodile.org/lord/>.
   *
   *  \param command_state Raw command to send to the USB interface.
   *  \return kConnected on success or kIoError on failure.
   */
  RoboticArmUsb::Status RoboticArmUsb::sendCommandState(Command command_state)
  {
    if(int error = libusb_control_transfer(libusb_device_handle_, 0x40, 0x06, 0x100, 0,
          (uint8_t *)&command_state, sizeof(command_state), 0) != sizeof(command_state)) {
      if(error < 0) {
        std::string message{"An error occured while sending a command to the robotic arm: "
          "libusb error: " + std::string(libusb_error_name(error))
          + " (" + std::to_string(error) + ")"};
        std::cerr << message << "." << std::endl;
      }
      else {
        std::string message{"An error occured while sending a command to the robotic arm: "
          + std::to_string(error) + " of "
          + std::to_string(sizeof(command_state)) + " bytes sent"};
        std::cerr << message << "." << std::endl;
      }
      return Status::kIoError;
    }
    return Status::kConnected;
  }

}
