/******************************************************************************
 * @brief        
 * 
 * @file         mod_controller.cpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-01
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#include "mod_controller.hpp"

namespace my_engineer {

CModController *pController_test;

/******************************************************************************
 * @brief    初始化控制器模块
 ******************************************************************************/
EAppStatus CModController::InitModule(SModInitParam_Base &param) {

	// 检查param是否正确
	if (param.moduleID == EModuleID::MOD_NULL) return APP_ERROR;

	// 类型转换
	auto controllerParam = static_cast<SModInitParam_Controller &>(param);
	moduleID = controllerParam.moduleID;

	// 初始化控制器组件
	comPitchEnd_.InitComponent(param);
	comYaw_.InitComponent(param);
	comPitch1_.InitComponent(param);
	comPitch2_.InitComponent(param);
	comRoll_.InitComponent(param);
	// comRocker_.InitComponent(param);
	comBuzzer_.InitComponent(param);


	// 创建任务并注册模块
	CreateModuleTask_();
	RegisterModule_();

	// test
	pController_test = this;

	Module_FSMFlag_ = FSM_RESET;
	moduleStatus = APP_OK;

	return APP_OK;
}

/******************************************************************************
 * @brief    更新处理
 ******************************************************************************/
void CModController::UpdateHandler_() {

	// 检查模块状态
	if (moduleStatus == APP_RESET) return ;

	// 更新组件
	comPitch1_.UpdateComponent();
	comPitch2_.UpdateComponent();
	comYaw_.UpdateComponent();
	comRoll_.UpdateComponent();
	comPitchEnd_.UpdateComponent();
	// comRocker_.UpdateComponent();
	comBuzzer_.UpdateComponent();


	// 更新模块信息
	// ControllerInfo.rocker_X = -static_cast<int8_t>(comRocker_.rockerInfo.X / (CONTROLLER_ROCKER_RANGE / 2.0f) * 100.0f);
	// ControllerInfo.rocker_Y = -static_cast<int8_t>(comRocker_.rockerInfo.Y / (CONTROLLER_ROCKER_RANGE / 2.0f) * 100.0f);
	// ControllerInfo.rocker_Key = comRocker_.rockerInfo.Key_status;
	ControllerInfo.posit_yaw = CModController::CComYaw::MtrPositToPhyPosit(comYaw_.yawInfo.posit);
	ControllerInfo.posit_pitch1 = comPitch1_.pitch1Info.posit;
	ControllerInfo.posit_pitch2 = comPitch2_.pitch2Info.posit;
	ControllerInfo.posit_roll = CModController::CComRoll::MtrPositToPhyPosit(comRoll_.rollInfo.posit);
	ControllerInfo.posit_pitch_end = CModController::CComPitchEnd::MtrPositToPhyPosit(comPitchEnd_.pitchEndInfo.posit);

	

	// 填充电机发送缓冲区
	CDevMtrDJI::FillCanTxBuffer(comRoll_.motor[0],
							   comRoll_.mtrCanTxNode_[0]->dataBuffer,
							   comRoll_.mtrOutputBuffer[0]);
	CDevMtrDJI::FillCanTxBuffer(comPitchEnd_.motor[0],
							   comPitchEnd_.mtrCanTxNode_[0]->dataBuffer,
							   comPitchEnd_.mtrOutputBuffer[0]);
	CDevMtrDJI::FillCanTxBuffer(comYaw_.motor[0],
							   comYaw_.mtrCanTxNode_[0]->dataBuffer,
							   comYaw_.mtrOutputBuffer[0]);

}

/******************************************************************************
 * @brief    心跳处理
 ******************************************************************************/
void CModController::HeartbeatHandler_() {

}

/******************************************************************************
 * @brief    创建控制器模块任务
 ******************************************************************************/
EAppStatus CModController::CreateModuleTask_() {

	// 任务已存在，删除任务
	if (moduleTaskHandle != nullptr) vTaskDelete(moduleTaskHandle);

	// 创建任务
	xTaskCreate(StartControllerModuleTask, "Controller Module Task",
						 512, this, proc_ModuleTaskPriority,
						  &moduleTaskHandle);

	return APP_OK;
}

/******************************************************************************
 * @brief    限制控制器模块的控制命令大小
 ******************************************************************************/
EAppStatus CModController::RestrictControllerCommand_() {
	
	// 检查模块状态
	if (moduleStatus == APP_RESET) {
		ControllerCmd = SControllerCmd();
		return APP_ERROR;
	}

	// 限制控制器各模块的控制命令大小
	ControllerCmd.cmd_yaw = 
		std::clamp(ControllerCmd.cmd_yaw, -CONTROLLER_YAW_PHYSICAL_RANGE_MIN, CONTROLLER_YAW_PHYSICAL_RANGE_MAX);
	ControllerCmd.cmd_pitch1 = 
		std::clamp(ControllerCmd.cmd_pitch1, 4.0f, CONTROLLER_PITCH1_PHYSICAL_RANGE);
	ControllerCmd.cmd_pitch2 =
		std::clamp(ControllerCmd.cmd_pitch2, 11.0f, CONTROLLER_PITCH2_PHYSICAL_RANGE);
	ControllerCmd.cmd_roll =
		std::clamp(ControllerCmd.cmd_roll, 0.0f, CONTROLLER_ROLL_PHYSICAL_RANGE);
	ControllerCmd.cmd_pitch_end =
		std::clamp(ControllerCmd.cmd_pitch_end, CONTROLLER_PITCH_END_PHYSICAL_RANGE_MIN, CONTROLLER_PITCH_END_PHYSICAL_RANGE_MAX);

	return APP_OK;

}


} // namespace my_engineer
