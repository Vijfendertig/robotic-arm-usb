#ifndef __VIJFENDERTIG__QT_CONTROL_UNIT__MAIN_WINDOW

  #define __VIJFENDERTIG__QT_CONTROL_UNIT__MAIN_WINDOW


  #include <iostream>
  #include <QtGui/QMainWindow>

  #include <robotic-arm-usb.h>

  #include <ui_qt-control-unit.h>


  namespace vijfendertig {

    class QtControlUnitWindow: public QMainWindow{

      Q_OBJECT

      public:

        QtControlUnitWindow(int argc, char ** argv, QWidget * parent = 0);
        virtual ~QtControlUnitWindow();

      public Q_SLOTS:

        void on_button_connect_clicked();
        void on_button_disconnect_clicked();
        void on_button_gripper_close_pressed();
        void on_button_gripper_close_released();
        void on_button_gripper_open_pressed();
        void on_button_gripper_open_released();
        void on_button_wrist_up_pressed();
        void on_button_wrist_up_released();
        void on_button_wrist_down_pressed();
        void on_button_wrist_down_released();
        void on_button_elbow_up_pressed();
        void on_button_elbow_up_released();
        void on_button_elbow_down_pressed();
        void on_button_elbow_down_released();
        void on_button_shoulder_up_pressed();
        void on_button_shoulder_up_released();
        void on_button_shoulder_down_pressed();
        void on_button_shoulder_down_released();
        void on_button_base_ccw_pressed();
        void on_button_base_ccw_released();
        void on_button_base_cw_pressed();
        void on_button_base_cw_released();
        void on_button_light_off_pressed();
        void on_button_light_on_pressed();

      private:

        Ui::main_window ui;
        RoboticArmUsb robotic_arm;

        void setStatusMessage(std::string status);
    };

  }

#endif // __VIJFENDERTIG__QT_CONTROL_UNIT__MAIN_WINDOW
