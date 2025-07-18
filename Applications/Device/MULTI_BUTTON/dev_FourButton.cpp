/*
 * @Description: 
 * @Author: Sassinak
 * @version: 
 * @Date: 2025-07-14 17:51:47
 * @LastEditors: Sassinak
 * @LastEditTime: 2025-07-17 22:43:00
 */
#include "multi_button\dev_FourButton.hpp"

namespace my_engineer {

CDevFourButton *pDev_button_test = nullptr;

bool CDevFourButton::isReset = false;
bool CDevFourButton::isLevel4 = false;
bool CDevFourButton::isLevel3 = false;
bool CDevFourButton::isSelf = false;
CDevFourButton::singlebutton CDevFourButton::buttons_[static_cast<int>(EButtonID::BUTTON_MAX)] = {};

uint8_t CDevFourButton::ButtonGpioRead(uint8_t button_id){
  if (button_id >= static_cast<uint8_t>(EButtonID::BUTTON_MAX)) {
    return 0;
  }
  switch(button_id){
    case EButtonID::RESET_BUTTON:
    case EButtonID::LEVEL_4_BUTTON:
    case EButtonID::LEVEL_3_BUTTON:
    case EButtonID::SELF_BUTTON:
      return HAL_GPIO_ReadPin(buttons_[button_id].halGpioPort, buttons_[button_id].halGpioPin);
  }

  return 0;
}

void CDevFourButton::ButtonPressDownCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  switch(button_id){
    case EButtonID::RESET_BUTTON:{
      break;
    }
    case EButtonID::LEVEL_4_BUTTON:{
      isLevel4 = true;
      break;
    }
    case EButtonID::LEVEL_3_BUTTON:{
      isLevel3 = true;
      break;
    }
    case EButtonID::SELF_BUTTON:{

      break;
    }
  }
}

void CDevFourButton::ButtonPressUpCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  switch(button_id){
    case EButtonID::RESET_BUTTON:{

      break;
    }
    case EButtonID::LEVEL_4_BUTTON:{
      isLevel4 = false;
      break;
    }
    case EButtonID::LEVEL_3_BUTTON:{
      isLevel3 = false;
      break;
    }
    case EButtonID::SELF_BUTTON:{

      break;
    }
  }
}

void CDevFourButton::ButtonLongPressCallback(void *btn) {
  Button* button = static_cast<Button *>(btn);
  uint8_t button_id = button->button_id;
  switch(button_id){
    case EButtonID::RESET_BUTTON:{
      isReset = true;
      break;
    }
    case EButtonID::LEVEL_4_BUTTON:{
      // HandleLevel4LongPress();
      break;
    }
    case EButtonID::LEVEL_3_BUTTON:{
      // HandleLevel3LongPress();
      break;
    }
    case EButtonID::SELF_BUTTON:{
      break;
    }
  }
}


/**
 * 
 */
EAppStatus CDevFourButton::InitDevice(const SDevInitParam_Base *pStructInitParam) {

  // 检查参数是否正确
  if (pStructInitParam == nullptr) return APP_ERROR;
  if (pStructInitParam->deviceID == EDeviceID::DEV_NULL) return APP_ERROR;

  // 类型转换
  auto initParam = static_cast<const SDevInitParam_FourButton *>(pStructInitParam);
  deviceID = initParam->deviceID;

  // 初始化每个按键
  for (int i = 0; i < static_cast<int>(EButtonID::BUTTON_MAX); ++i) {
    buttons_[i].buttonID = initParam->buttons_[i].buttonID;
    buttons_[i] = initParam->buttons_[i];

    button_init(&buttons_[i].User_button, ButtonGpioRead, buttons_[i].activeLevel, static_cast<uint8_t>(buttons_[i].buttonID));
    button_attach(&buttons_[i].User_button,PressEvent::PRESS_DOWN,ButtonPressDownCallback);
    button_attach(&buttons_[i].User_button,PressEvent::PRESS_UP,ButtonPressUpCallback);
    button_attach(&buttons_[i].User_button,PressEvent::LONG_PRESS_HOLD,ButtonLongPressCallback);

    button_start(&buttons_[i].User_button);
  }
  RegisterDevice_();

  deviceStatus = APP_OK;
  pDev_button_test = this;
  return APP_OK; 
}

void CDevFourButton::UpdateHandler_() {
  // 调用父类的按键扫描
  button_ticks();
}

}
