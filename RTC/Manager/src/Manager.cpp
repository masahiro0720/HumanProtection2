// -*- C++ -*-
/*!
 * @file  Manager.cpp
 * @brief Manager
 * @date $Date$
 *
 * $Id$
 */

#include "Manager.h"

// Module specification
static const char* manager_spec[] =
  {
    "implementation_id", "Manager",
    "type_name",         "Manager",
    "description",       "Manager",
    "version",           "1.0.0",
    "vendor",            "rsdlab",
    "category",          "Manager",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    ""
  };

Manager::Manager(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_safetyIn("safety", m_safety),
    m_end_moveIn("end_move", m_end_move),
    m_end_manipIn("end_manip", m_end_manip),
    m_stopOut("stop", m_stop),
    m_start_moveOut("start_move", m_start_move),
    m_ManipulatorCommonInterface_CommonPort("ManipulatorCommonInterface_Common"),
    m_ManipulatorCommonInterface_MiddlePort("ManipulatorCommonInterface_Middle")
{
}

Manager::~Manager()
{
}

RTC::ReturnCode_t Manager::onInitialize()
{
  addInPort("safety", m_safetyIn);
  addInPort("end_move", m_end_moveIn);
  addInPort("end_manip", m_end_manipIn);

  addOutPort("stop", m_stopOut);
  addOutPort("start_move", m_start_moveOut);

  m_ManipulatorCommonInterface_CommonPort.registerConsumer("ManipulatorCommonInterface_Common", "JARA_ARM::ManipulatorCommonInterface_Common", m_ManipulatorCommonInterface_Common);
  m_ManipulatorCommonInterface_MiddlePort.registerConsumer("ManipulatorCommonInterface_Middle", "JARA_ARM::ManipulatorCommonInterface_Middle", m_ManipulatorCommonInterface_Middle);

  addPort(m_ManipulatorCommonInterface_CommonPort);
  addPort(m_ManipulatorCommonInterface_MiddlePort);

  return RTC::RTC_OK;
}

RTC::ReturnCode_t Manager::onActivated(RTC::UniqueId ec_id)
{
  sleep(1);
  std::cout << "Manager Activated: Sequence Loop Start." << std::endl;
  
  phase = 0;          
  wait_timer = 0;     
  was_danger = false; 

  // --- 座標データの定義 ---
  // Pick1姿勢
  Pick1Point.length(6);
  Pick1Point[0] = 0.000;
  Pick1Point[1] = -0.098;
  Pick1Point[2] = 0.233;
  Pick1Point[3] = 0.000;
  Pick1Point[4] = 1.385;
  Pick1Point[5] = 0.000;

  // Place姿勢
  PlacePoint.length(6);
  PlacePoint[0] = -1.555;
  PlacePoint[1] = -1.181;
  PlacePoint[2] = 0.673;
  PlacePoint[3] = 0.000;
  PlacePoint[4] = 0.609;
  PlacePoint[5] = 0.000;

  // Pick2姿勢
  Pick2Point.length(6);
  Pick2Point[0] = -1.555;
  Pick2Point[1] = 0.006;
  Pick2Point[2] = 0.135;
  Pick2Point[3] = 0.000;
  Pick2Point[4] = 1.486;
  Pick2Point[5] = 0.000;

  return RTC::RTC_OK;
}

RTC::ReturnCode_t Manager::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

// 動作指令を送信するヘルパー関数
void Manager::sendCurrentMotion()
{
    JARA_ARM::JointPos targetPoint;
    
    switch(phase) {
        case 0: 
            std::cout << ">>> Move to Pick1." << std::endl;
            targetPoint = Pick1Point;
            break;
        case 1: 
            std::cout << ">>> Move to Place." << std::endl;
            targetPoint = PlacePoint;
            break;
        case 2: 
            std::cout << ">>> Move to Pick2." << std::endl;
            targetPoint = Pick2Point;
            break;
        case 3: 
            std::cout << ">>> Move to Place." << std::endl;
            targetPoint = PlacePoint;
            break;
        default:
            phase = 0;
            targetPoint = Pick1Point;
            break;
    }
    m_ManipulatorCommonInterface_Middle->movePTPJointAbs(targetPoint);
}

RTC::ReturnCode_t Manager::onExecute(RTC::UniqueId ec_id)
{
  if(m_safetyIn.isNew())
  {
    m_safetyIn.read();
  }

  // ============================================================
  // 1. 危険検知時 (safety != 0) -> 強制停止
  // ============================================================
  if(m_safety.data != 0)
  {
    // 停止信号（ありえない値 999.0）を送信して、ブリッジ側で急停止させる
    JARA_ARM::JointPos stopCmd;
    stopCmd.length(6);
    for(int i=0; i<6; i++) stopCmd[i] = 999.0;
    m_ManipulatorCommonInterface_Middle->movePTPJointAbs(stopCmd);

    // ログ表示
    // std::cout << "DANGER DETECTED! SENDING STOP SIGNAL (999.0)." << std::endl;
    
    m_stop.data = "1";
    m_stopOut.write();
    was_danger = true;
    
    return RTC::RTC_OK; 
  }
  // ============================================================
  // 2. 安全時 (safety == 0) -> 動作継続 / 再開
  // ============================================================
  else 
  {
    m_stop.data = "0";
    m_stopOut.write();

    // ★復帰処理★
    if (was_danger)
    {
        std::cout << "Safety Restored. RESUMING Current Motion." << std::endl;
        sendCurrentMotion(); // 中断していた動作を再送信
        was_danger = false;
    }

    if (wait_timer > 0) {
        wait_timer--;
        return RTC::RTC_OK;
    }

    // 待機終了、次の動作へ
    sendCurrentMotion();

    phase++;
    if (phase > 3) phase = 0;

    // 3秒待機 (800Hz換算で2400)
    wait_timer = 2400; 
  }

  return RTC::RTC_OK;
}

extern "C"
{
  void ManagerInit(RTC::Manager* manager)
  {
    coil::Properties profile(manager_spec);
    manager->registerFactory(profile,
                             RTC::Create<Manager>,
                             RTC::Delete<Manager>);
  }
};
