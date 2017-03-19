//! Implementation of the main window for the Velleman/OWI Robotic Arm's virtual control unit.
/*!
 *  \file
 *  \author Maarten De Munck, <maarten@vijfendertig.be>
 *  \date 2017
 *  \copyright Licensed under the MIT License. See LICENSE for the full license.
 */


#include "../include/qt-control-unit-window.h"


namespace vijfendertig {

  QtControlUnitWindow::QtControlUnitWindow(int argc, char ** argv, QWidget * parent):
    QMainWindow(parent),
    robotic_arm{}
  {
    ui.setupUi(this);
    setFixedSize(size());
  }

  QtControlUnitWindow::~QtControlUnitWindow()
  {
    robotic_arm.disconnect();
  }

  void QtControlUnitWindow::on_button_connect_clicked()
  {
    auto status = robotic_arm.connect();
    if(status == RoboticArmUsb::Status::kConnected) {
      if(ui.button_light_off->isChecked()) {
        robotic_arm.sendCommand(RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOff);
      }
      if(ui.button_light_on->isChecked()) {
        robotic_arm.sendCommand(RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOn);
      }
    }
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_disconnect_clicked()
  {
    auto status = robotic_arm.disconnect();
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_gripper_close_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kGripper, RoboticArmUsb::Action::kClose);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_gripper_close_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kGripper, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_gripper_open_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kGripper, RoboticArmUsb::Action::kOpen);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_gripper_open_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kGripper, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_wrist_up_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kWrist, RoboticArmUsb::Action::kUp);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_wrist_up_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kWrist, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_wrist_down_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kWrist, RoboticArmUsb::Action::kDown);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_wrist_down_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kWrist, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_elbow_up_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kElbow, RoboticArmUsb::Action::kUp);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_elbow_up_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kElbow, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_elbow_down_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kElbow, RoboticArmUsb::Action::kDown);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_elbow_down_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kElbow, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_shoulder_up_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kShoulder, RoboticArmUsb::Action::kUp);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_shoulder_up_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kShoulder, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_shoulder_down_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kShoulder, RoboticArmUsb::Action::kDown);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_shoulder_down_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kShoulder, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_base_ccw_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kBase, RoboticArmUsb::Action::kCCW);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_base_ccw_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kBase, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_base_cw_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kBase, RoboticArmUsb::Action::kCW);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_base_cw_released()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kBase, RoboticArmUsb::Action::kStop);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_light_off_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOff);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::on_button_light_on_pressed()
  {
    auto status = robotic_arm.sendCommand(
        RoboticArmUsb::Actuator::kLight, RoboticArmUsb::Action::kOn);
    setStatusMessage(robotic_arm.getStatusString(status));
  }

  void QtControlUnitWindow::setStatusMessage(std::string message)
  {
    message[0] = std::toupper(message[0]);
    ui.label_status->setText(QString::fromStdString(message));
  }

}
