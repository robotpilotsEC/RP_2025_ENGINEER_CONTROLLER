/******************************************************************************
 * @brief        
 * 
 * @file         dev_controller_link.hpp
 * @author       Fish_Joe (2328339747@qq.com)
 * @version      V1.0
 * @date         2025-04-05
 * 
 * @copyright    Copyright (c) 2025
 * 
 ******************************************************************************/

#ifndef DEV_CONTROLLER_LINK_HPP
#define DEV_CONTROLLER_LINK_HPP

#include "dev_common.hpp"

#include "inf_uart.hpp"
#include "algo_crc.hpp"

namespace my_engineer {

/**
 * @brief 控制器通信设备类
 * 
 */
class CDevControllerLink final: public CDevBase {
public:

	// 定义控制器通信设备初始化参数结构体
	struct SDevInitParam_ControllerLink: public SDevInitParam_Base{
		EInterfaceID interfaceID = EInterfaceID::INF_NULL; ///< 串口ID
	};

	enum EPackageID: uint8_t {
		ID_NULL = 0,
		ID_CONTROLLER_DATA,
		ID_ROBOT_DATA,
		ID_CHOSELEVEL_DATA,
	};

	struct SPkgHeader {
		uint8_t SOF = 0xA5; ///< 包头
		uint16_t pkgLen = 0; ///< 包长度
		uint8_t seq = 0; ///< 包序号
		uint8_t CRC8 = 0x00; ///< CRC8校验
		uint16_t cmd_Id = 0x0000; ///< 命令ID
	} __packed; //禁止编译器的内存对齐优化

	struct SControllerDataPkg {
		SPkgHeader header;
		bool controller_OK = 0; ///< 控制器状态
		bool return_success = 0; ///< 归位成功标志
		/*-----------maybe it will use-----------------*/
		// int8_t rocker_X = 0; ///< 摇杆X轴值
		// int8_t rocker_Y = 0; ///< 摇杆Y轴值
		// uint8_t rocker_Key = 0; ///< 摇杆按键状态
		float_t angle_yaw = 0; ///< yaw角度位置
		float_t angle_pitch1 = 0; ///< Pitch1角度位置
		float_t angle_pitch2 = 0; ///< Pitch2角度位置
		float_t angle_roll = 0; ///< roll电机位置
		float_t angle_pitch_end = 0; ///< 末端Pitch角度位置
		float_t angle_roll_end = 0; ///< 末端Roll角度位置
		int8_t reserved[4] = {0}; ///< 保留字段
		uint16_t CRC16 = 0x0000; ///< CRC16校验
	} __packed controllerData_info_pkg = { };

	struct SRobotDataPkg {
		SPkgHeader header;
		bool ask_reset_flag = 0; ///< 要求复位标志
		bool controlled_by_controller = 0; ///< 是否被控制器控制
		bool ask_return_flag = 0; ///< 要求归位标志
		float_t angle_yaw = 0; ///< yaw角度位置
		float_t angle_pitch1 = 0; ///< Pitch1角度位置
		float_t angle_pitch2 = 0; ///< Pitch2角度位置
		float_t angle_roll = 0; ///< roll电机位置
		float_t angle_pitch_end = 0; ///< 末端Pitch角度位置
		float_t angle_roll_end = 0; ///< 末端Roll角度位置
		int8_t reserved[3] = {0}; ///< 保留字段
		uint16_t CRC16 = 0x0000; ///< CRC16校验
	} __packed robotData_info_pkg = { };

	/*for chose level*/
	struct SChoseLevelDataPkg {
		SPkgHeader header;
		uint8_t Key_value1;
		uint8_t Key_value2;
		uint16_t x_position:12;
    uint16_t mouse_left:4;
    uint16_t y_position:12;
    uint16_t mouse_right:4;
		int8_t reserved[1] = {0}; 
		uint16_t CRC16 = 0x0000;
	}__packed choseLevelData_info_pkg = { };
	

	enum class EControllerLinkStatus {
		RESET,
		OFFLINE,
		ONLINE,
	} controllerLinkStatus = EControllerLinkStatus::RESET;

	/*Please increase or decrease difficulty based on the season  */
	enum class EOreDifficultyLevel {
		NONE,
		FIRST,
		SECOND,
		THIRD,
		FOURTH,
	} oreDifficultyLevel = EOreDifficultyLevel::NONE;

	CDevControllerLink() { deviceType = EDevType::DEV_CONTROLLER_LINK; }

	EAppStatus InitDevice(const SDevInitParam_Base *pStructInitParam) override;

	EAppStatus SendPackage(EPackageID packageID, SPkgHeader &packageHeader);

private:

	CInfUART *uartInterface_ = nullptr; ///< 串口设备指针

	std::array<uint8_t, 512> rxBuffer_ = {0};

	uint32_t rxTimestamp_ = 0; ///< 接收时间戳

	void UpdateHandler_() override;

	void HeartbeatHandler_() override;

	EAppStatus ResolveRxPackage_();
};

/*--------------------------------Key-Value Mapping Table---------------------------------------------*/
#define A_KEY_VALUE 65
#define B_KEY_VALUE 66
#define C_KEY_VALUE 67
#define D_KEY_VALUE 68
#define E_KEY_VALUE 69
#define F_KEY_VALUE 70
#define G_KEY_VALUE 71
#define H_KEY_VALUE 72
#define I_KEY_VALUE 73
#define J_KEY_VALUE 74
#define K_KEY_VALUE 75
#define L_KEY_VALUE 76
#define M_KEY_VALUE 77
#define N_KEY_VALUE 78
#define O_KEY_VALUE 79
#define P_KEY_VALUE 80
#define Q_KEY_VALUE 81
#define R_KEY_VALUE 82
#define S_KEY_VALUE 83
#define T_KEY_VALUE 84
#define U_KEY_VALUE 85
#define V_KEY_VALUE 86
#define W_KEY_VALUE 87
#define X_KEY_VALUE 88
#define Y_KEY_VALUE 89
#define Z_KEY_VALUE 90

} // namespace my_engineer

#endif // DEV_CONTROLLER_LINK_HPP
