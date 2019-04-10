#include "control.h"

/*****************************************************************************
  函数名称:       void SetAngular_TestDemo( void )
  说明:           设置目标方向盘转角信号
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAngular接口   
*****************************************************************************/
void SetAngular_BYD(double inputAngle)
{
    //double  inputAngle = 0.0;
    uint16_t nAngular = 0;
    int32_t modValue = 0;

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Valid angle: [-500.0, +500.0]°.\n");
    printf("SetAngular.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    //sem_wait(&sem_event);

    modValue = (int32_t)(inputAngle * 10) % 10;
    inputAngle = (int32_t)inputAngle + modValue * 0.1;

    printf("angle: %.2f.\n", inputAngle);
    nAngular = (780 + inputAngle) * 10;

    if (ERR_BYD_AUTO_OK == SetAngular(nAngular)) {
        printf("nAngular: 0x%x.\n", nAngular);
    }
    else {
        printf("call SetAngular() Error.\n");
    }
    printf("\n");
    
}

/*****************************************************************************
  函数名称:       void SetAcceleratedVelocity_TestDemo( void )
  说明:           设置目标加速度信息
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetAcceleratedVelocity接口   
*****************************************************************************/
void SetAcceleratedVelocity_BYD( double inputAccVal )
{
    //double inputAccVal = 0.0;
    uint8_t nAcceleratedVelocity = 0;
    int32_t modValue = 0;

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("Valid acclerated-Velocity: [-5.00, +5.00]m/s^2.\n");
    printf("SetAcceleratedVelocity.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    //sem_wait(&sem_event_SetAcceleratedVelocity);

        modValue = (int32_t)(inputAccVal * 10) % 10;
        inputAccVal = (int32_t)inputAccVal + modValue * 0.1;
        printf("acclerated-velocity: %.2f.\n", inputAccVal);

        // --------------------------------------
        if (inputAccVal < -5 || inputAccVal > 5) {
            printf("Error: out of range.\n");
            return;
        }
        nAcceleratedVelocity = (inputAccVal + 5) * 20;

        if (ERR_BYD_AUTO_OK == SetAcceleratedVelocity(nAcceleratedVelocity)) {
            printf("nAcceleratedVelocity: 0x%x.\n", nAcceleratedVelocity);
        }
        else {
            printf("call SetAcceleratedVelocity() Error.\n");
        }
        
        printf("\n");
    
}

/*****************************************************************************
  函数名称:       void SetCarGear_TestDemo( void )
  说明:           设置目标档位
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetCarGear接口   
*****************************************************************************/
void SetCarGear_BYD( uint8_t CarGear_val )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("SetCarGear.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
	printf("CarGear_val is: %d\n",CarGear_val);
        //sem_wait(&sem_event);
            switch (CarGear_val) {
            case 0: {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_INVALID)) {
                        printf("BYD_AUTO_CAR_GEAR_INVALID: Invalid Gear signal.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case 1: {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_P)) {
                        printf("BYD_AUTO_CAR_GEAR_P: Gear-P.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case 2: {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_R)) {
                        printf("BYD_AUTO_CAR_GEAR_R: Gear-R.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case 3: {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_N)) {
                        printf("BYD_AUTO_CAR_GEAR_N: Gear-N.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            case 4: {
                    if (ERR_BYD_AUTO_OK == SetCarGear(BYD_AUTO_CAR_GEAR_D)) {
                        printf("BYD_AUTO_CAR_GEAR_D: Gear-D.\n");
                    }
                    else {
                        printf("call SetCarGear() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }

        printf("\n");
    
}
/*****************************************************************************
  函数名称:       void SetEpbState_TestDemo( void )
  说明:           设置目标EPB状态
  调用:
  被调用:
  输入参数:       void	
  输出参数:       
  返回值:         void
  被访问变量:
  被修改变量:
  其他:           调用SetEpbState接口   
*****************************************************************************/
void SetEpbState_BYD( uint8_t CarEpb_val )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("SetEPB.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
        //sem_wait(&sem_event);
        switch (CarEpb_val) {
            case 0: {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_INVALID)) {
                        printf("BYD_AUTO_EPB_SET_INVALID: Epb Invalid request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case 1: {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_APPLYING)) {
                        printf("BYD_AUTO_EPB_SET_APPLYING: Epb Applying request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            case 2: {
                    if (ERR_BYD_AUTO_OK == SetEpbState(BYD_AUTO_EPB_SET_RELEASED)) {
                        printf("BYD_AUTO_EPB_SET_RELEASED: Epb Released request.\n");
                    }
                    else {
                        printf("call SetEpbState() Error.\n");
                    }
                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        
        printf("\n");
    
}

void SetDrivingMode_BYD(uint8_t DrivingMode_val)
{
    Get_EPS_ESP_Permission_BYD(3);

    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("SetDrivingMode.\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    printf("DrivingMode_val is %d\n",DrivingMode_val);
            switch (DrivingMode_val) {
            case 0: {
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING)) {
                        printf("BYD_AUTO_DRIVE_MODE_HOLDING: Hold current drive mode.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                }
                break;
            case 1: {
                    //向各功能模块发送驾驶模式请求开启
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_OPEN: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                    //横向控制开启
                    if (ERR_BYD_AUTO_OK == SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_EPS_OPEN: EPS AutoDrive ON.\n");
                    }
                    else {
                        printf("call SetAutoDriveLateralState() Error.\n");
                    }
                    //纵向控制开启
                    if (ERR_BYD_AUTO_OK == SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_STATE_OPEN: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SetDriveAutoState() Error.\n");
                    }

                    if((GetEPSDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO) &&
                    (GetESPDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN) &&
                    (GetMCDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO) &&
                    (GetSwitchDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO) &&
                    (GetGearDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO) &&
                    (GetEleBrakeDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_AUTO)
                    ){

                        SetDriveModeFeedback_BYD(BYD_AUTO_DRIVE_MODE_AUTO);//仪表绿色脱手标志
                        SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING);
                        printf("SetDrivingMode Auto success!\n");
                    }
                }
                break;
            case 2: {
                    //向各功能模块发送驾驶模式请求关闭
                    if (ERR_BYD_AUTO_OK == SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_CLOSE: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SendAutoDriveModeRequest() Error.\n");
                    }
                    //横向控制关闭
                    if (ERR_BYD_AUTO_OK == SetAutoDriveLateralState(BYD_AUTO_DRIVE_MODE_EPS_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_EPS_CLOSE: EPS AutoDrive ON.\n");
                    }
                    else {
                        printf("call SetAutoDriveLateralState() Error.\n");
                    }
                    //纵向控制关闭
                    if (ERR_BYD_AUTO_OK == SetDriveAutoState(BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE)) {
                        printf("BYD_AUTO_DRIVE_MODE_AUTO_STATE_CLOSE: Auto-drive mode request.\n");
                    }
                    else {
                        printf("call SetDriveAutoState() Error.\n");
                    }
                    if((GetEPSDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL) &&
                    (GetESPDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE) &&
                    (GetMCDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL) &&
                    (GetSwitchDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL) &&
                    (GetGearDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL) &&
                    (GetEleBrakeDriveModeFeedbackInfo_BYD() == BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_NORMAL)
                    ){
                        SetDriveModeFeedback_BYD(BYD_AUTO_DRIVE_MODE_NORMAL);//仪表白色手握方向盘
                        SendAutoDriveModeRequest(BYD_AUTO_DRIVE_MODE_HOLDING);
                        printf("SetDrivingMode normal success!\n");
                    }

                }
                break;
            default:
                printf("this input value is not supported.\n");
                break;
            }
        printf("\n");
}

void Get_EPS_ESP_Permission_BYD(uint8_t err_count){
    //GetDriveAutoPermissionInfo
    uint8_t epsAutoPermission = 0;
    uint64_t epsTimeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetDriveAutoPermissionInfo(&epsAutoPermission, &epsTimeStamp)) {
        printf("*pnDriveAutoPermission: 0x%x, *pnepsTimeStamp: 0x%lx.\n", epsAutoPermission, epsTimeStamp);

        switch (epsAutoPermission) {
        case BYD_AUTO_EPS_PERMISSION_INVALID: {
                printf("EPS auto drive permission signal invalid.\n");
            }
            break;
        case BYD_AUTO_EPS_PERMISSION_OK: {
                printf("EPS auto drive permission.\n");
            }
            break;
        case BYD_AUTO_EPS_NOT_PERMISSION: {
                printf("EPS auto drive not permission.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetDriveAutoPermissionInfo() Error.\n");
    }
    printf("\n");

    //GetESPAutoDrivePermissionInfo
    uint8_t espPermissionInfo = 0;
    uint64_t espTimeStamp = 0;
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetESPAutoDrivePermissionInfo(&espPermissionInfo, &espTimeStamp)) {
        printf("*pnESPDriveModeFeedback: 0x%x, *pnespTimeStamp: 0x%lx.\n", espPermissionInfo, espTimeStamp);

        switch (espPermissionInfo) {
        case BYD_AUTO_DRIVE_ESP_NOT_PERMISSION: {
                printf("ESP Auto Drive Mode is not allowable.\n");
            }
            break;
        case BYD_AUTO_DRIVE_ESP_PERMISSION_OK: {
                printf("ESP Auto Drive Mode is allowable.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetESPAutoDrivePermissionInfo() Error.\n");
    }
    printf("\n");

    if((espPermissionInfo == BYD_AUTO_DRIVE_ESP_PERMISSION_OK)&&(epsAutoPermission == BYD_AUTO_EPS_PERMISSION_OK)){
        printf("EPS auto drive permission.\n");         
        printf("ESP Auto Drive Mode is allowable.\n");
        //return 0;
    }
    else{
        err_count--;
        if (err_count == 0){
            printf("Get EPS ESP permission failed.\n");   

        }
        else{
            return Get_EPS_ESP_Permission_BYD(err_count);
        }
    }
}
uint8_t GetEPSDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEPSDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnEPSDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_INVALID: {
                printf("EPS feedback signal invalid.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_NORMAL: {
                printf("EPS normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_EPS_FEEDBACK_AUTO: {
                printf("EPS auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEPSDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}

uint8_t GetESPDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetESPDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnESPDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_CLOSE: {
                printf("ESP normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ESP_FEEDBACK_OPEN: {
                printf("ESP auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetESPDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}

uint8_t GetMCDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetMCDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnMCDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_NORMAL: {
                printf("Motor controller normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_MC_FEEDBACK_AUTO: {
                printf("Motor controller auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetMCDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}

uint8_t GetSwitchDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetSwitchDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnSwitchDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_INVALID: {
                printf("invalid signal.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_NORMAL: {
                printf("normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_SWITCH_FEEDBACK_AUTO: {
                printf("auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetSwitchDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}

uint8_t GetGearDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetGearDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnGearDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_NORMAL: {
                printf("Gear normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_GEAR_FEEDBACK_AUTO: {
                printf("Gear auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetGearDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}

uint8_t GetEleBrakeDriveModeFeedbackInfo_BYD( void )
{
    uint8_t driveModeFeedback = 0;
    uint64_t timeStamp = 0;

    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetEleBrakeDriveModeFeedbackInfo(&driveModeFeedback, &timeStamp)) {
        printf("*pnEleBrakeDriveModeFeedback: 0x%x, *pnTimeStamp: 0x%lx.\n", driveModeFeedback, timeStamp);

        switch (driveModeFeedback) {
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_INVALID: {
                printf("EPB feedback signal invalid.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_NORMAL: {
                printf("EPB normal drive mode.\n");
            }
            break;
        case BYD_AUTO_DRIVE_MODE_ELE_BRAKE_FEEDBACK_AUTO: {
                printf("EPB auto drive mode.\n");
            }
            break;
        default:
            break;
        }
    }
    else {
        printf("call GetEleBrakeDriveModeFeedbackInfo() Error.\n");
    }
    printf("\n");
    return driveModeFeedback;
}
void SetDriveModeFeedback_BYD( uint8_t DriveModeFeedback_val )
{
    printf("\n");
    printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    switch (DriveModeFeedback_val) {
    case 0: {
        if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1)) {
            printf("BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_1: Auto-drive abnormal-1.\n");
        }
        else {
            printf("call SetDriveModeFeedback() Error.\n");
        }
    }
    break;
    case 1: {
        if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO)) {
            printf("BYD_AUTO_DRIVE_MODE_AUTO: Auto-drive mode.\n");
        }
        else {
            printf("call SetDriveModeFeedback() Error.\n");
        }
    }
    break;
    case 2: {
        if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_NORMAL)) {
            printf("BYD_AUTO_DRIVE_MODE_NORMAL: Manual-drive mode.\n");
            }
        else {
            printf("call SetDriveModeFeedback() Error.\n");
        }
    }
    break;
    case 3: {
        if (ERR_BYD_AUTO_OK == SetDriveModeFeedback(BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2)) {
            printf("BYD_AUTO_DRIVE_MODE_AUTO_ABNORMAL_2: Auto-drive abnormal-2.\n");
        }
        else {
            printf("call SetDriveModeFeedback() Error.\n");
        }
    }
    break;
    default:
        printf("this input value is not supported.\n");
        break;
    }
    printf("\n");
    
}


void GetCarInfo_BYD(union sigval v)
{

    uint16_t currVelocity = 0;
    uint64_t timeStamp = 0;
    ////////////////realSpeed_val
    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetVelocityInfo(&currVelocity, &timeStamp)) {       
        realSpeed_val = (double)(currVelocity * 0.06875 + 0.5);
        realSpeed_val = realSpeed_val/3.6;
        //printf("*pnVelocity: 0x%x, *pnTimeStamp: 0x%lx.\n", currVelocity, timeStamp);
        //printf("real vehicle speed: %f.\n", realSpeed_val);
    }
    else {
        //printf("call GetVelocityInfo() Error.\n");
    }
    //printf("\n");



    uint16_t angleValue = 0;
    ////////////////realAngleInfo_val
    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAngularInfo(&angleValue, &timeStamp)) {
        //printf("*pnAngular: 0x%x, *pnTimeStamp: 0x%lx.\n", angleValue, timeStamp);

        realAngleInfo_val = (int16_t)angleValue * 0.1;
        //printf("realAngleInfo: %lf.\n", realAngleInfo_val);
		if(AUTO_MODE == driving_mode)
		{
			if(500 < realAngleInfo_val || realAngleInfo_val < -500)	
			{
				//printf("The realAngleInfo = %lf is out the range [-500, 500]!,"
					   //"the EPS may exit into nomal drive!\n",realAngleInfo_val);
			}
		}
    }
    else {
        //printf("call GetAngularInfo() Error.\n");
    }
    //printf("\n");




    uint16_t accXInfo = 0,
             accXInfoOffset = 0;
    double realAccXInfo = 0.0,
           realAccXInfoOffset = 0.0;
    /////////////wholeAccXInfo_val 纵向加速度
    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAccelerationXInfo(&accXInfo, &accXInfoOffset, &timeStamp)) {
        realAccXInfo = accXInfo * 0.027126736 - 21.593;
        realAccXInfoOffset = accXInfoOffset * 0.027126736 - 21.593;
        wholeAccXInfo_val = realAccXInfo - realAccXInfoOffset;
        //printf("*pnAX: 0x%x, *pnAXOffset: 0x%x, *pnTimeStamp: 0x%lx.\n", accXInfo, accXInfoOffset, timeStamp);
        //printf("realAccXInfo: %lf, realAccXInfoOffset: %lf, wholeAccXInfo: %lf.\n", realAccXInfo, realAccXInfoOffset, wholeAccXInfo_val);
    }
    else {
        //printf("call GetAccelerationXInfo() Error.\n");
    }
    //printf("\n");



    //////////accDeepness_val  油门踏板深度
    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAccelerateDeepnessInfo(&accDeepness_val, &timeStamp)) {

        //printf("*pnAccelerateDeepness: 0x%x, *pnTimeStamp: 0x%lx.\n", accDeepness_val, timeStamp);
        //printf("Acceleration deepness: %d percent.\n", accDeepness_val);
    }
    else {
        //printf("call GetAccelerateDeepnessInfo() Error.\n");
    }
    //printf("\n");

    ////////////////brakeDeepness_val  刹车踏板深度


    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetBrakeDeepnessInfo(&brakeDeepness_val, &timeStamp)) {
        
        //printf("*pnBrakeDeepness: 0x%x, *pnTimeStamp: 0x%lx.\n", brakeDeepness_val, timeStamp);
        //printf("Brake deepness: %d percent.\n", brakeDeepness_val);
    }
    else {
        //printf("call GetBrakeDeepnessInfo() Error.\n");
    }
    //printf("\n");
    
    //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetBrakePedalSignalInfo(&brakePedalStatus, &timeStamp)) {
        //printf("*pnBrakePedalSignal: 0x%x, *pnTimeStamp: 0x%lx.\n", brakePedalStatus, timeStamp);

        switch (brakePedalStatus) {
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_NO_PRESSED: {
                //printf("brake pedal is not pressed.\n");
            }
            break;
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_PRESSED: {
                //printf("brake pedal is pressed.\n");
            }
            break;
        case BYD_AUTO_BRAKE_PEDAL_SIGNAL_ERR: {
                //printf("brake pedal signal error.\n"); 
            }
            break;
        default:
            break;
        }
    }
    else {
        //printf("call GetBrakePedalSignalInfo() Error.\n");
    }
    //printf("\n");

        //printf("-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+-+\n");
    if (ERR_BYD_AUTO_OK == GetAutoDriveKeyInfo(&autoDriveKey, &timeStamp)) {
        //printf("*pnAutoDriveKey: 0x%x, *pnTimeStamp: 0x%lx.\n", autoDriveKey, timeStamp); 
    }


    sem_post(&sem_event_GetCarInfo);



}




