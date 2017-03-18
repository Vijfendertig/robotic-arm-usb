#include <robotic-arm-usb.h>
#include <chrono>
#include <iostream>
#include <thread>


using namespace vijfendertig;


int main(int argc, char ** argv)
{
  RoboticArmUsb robotic_arm;
  RoboticArmUsb::Status status;

  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::cerr << std::endl;
  // Connect.
  std::cerr << "connect" << std::endl;
  status = robotic_arm.connect();
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Connect again. This should notice we're already connected and just return 'connected'.
  std::cerr << "connect     (again, should be ignored if the previous call succeeded)" << std::endl;
  status = robotic_arm.connect();
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Turn on the LED.
  std::cerr << "sendCommand (LED on)" << std::endl;
  status = robotic_arm.sendCommand(RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOn);
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Turn off the LED.
  std::cerr << "sendCommand (LED off)" << std::endl;
  status = robotic_arm.sendCommand(RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOff);
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Turn on the LED.
  std::cerr << "sendCommand (LED on)" << std::endl;
  status = robotic_arm.sendCommand(RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOn);
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Disconnect. This should turn off the LED too. 
  std::cerr << "disconnect  (and turn LED off on disconnect)" << std::endl;
  status = robotic_arm.disconnect();
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cerr << std::endl;
  // Disconnect again. This should notice we're not connected and just return 'disconnected'.
  std::cerr << "disconnect  (again, should be ignored)" << std::endl;
  status = robotic_arm.disconnect();
  std::cerr << "            ==> '" << robotic_arm.getStatusString(status) << "'" << std::endl;
  std::cerr << "getState    ==> '" << robotic_arm.getStatusString(robotic_arm.getStatus()) << "'" << std::endl;

  return EXIT_SUCCESS;
}
