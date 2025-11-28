// -*- C++ -*-
/*!
 * @file  Manager.h
 * @brief Manager
 * @date  $Date$
 *
 * $Id$
 */

#ifndef MANAGER_H
#define MANAGER_H

#include <rtm/idl/BasicDataTypeSkel.h>
#include <rtm/idl/ExtendedDataTypesSkel.h>
#include <rtm/idl/InterfaceDataTypesSkel.h>

// Service Consumer stub headers
// <rtc-template block="consumer_stub_h">
#include "ManipulatorCommonInterface_CommonStub.h"
#include "ManipulatorCommonInterface_MiddleLevelStub.h"
#include "BasicDataTypeStub.h"

// </rtc-template>

#include <rtm/Manager.h>
#include <rtm/DataFlowComponentBase.h>
#include <rtm/CorbaPort.h>
#include <rtm/DataInPort.h>
#include <rtm/DataOutPort.h>

#include <string>
#include <vector>

/*!
 * @class Manager
 * @brief Manager
 *
 */
class Manager
  : public RTC::DataFlowComponentBase
{
 public:
  /*!
   * @brief constructor
   * @param manager Maneger Object
   */
  Manager(RTC::Manager* manager);

  /*!
   * @brief destructor
   */
  ~Manager();

  virtual RTC::ReturnCode_t onInitialize();
  virtual RTC::ReturnCode_t onActivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onDeactivated(RTC::UniqueId ec_id);
  virtual RTC::ReturnCode_t onExecute(RTC::UniqueId ec_id);


 protected:
  // DataInPort declaration
  // <rtc-template block="inport_declare">
  RTC::TimedBoolean m_safety;
  RTC::InPort<RTC::TimedBoolean> m_safetyIn;
  RTC::TimedString m_end_move;
  RTC::InPort<RTC::TimedString> m_end_moveIn;
  RTC::TimedString m_end_manip;
  RTC::InPort<RTC::TimedString> m_end_manipIn;
  // </rtc-template>

  // DataOutPort declaration
  // <rtc-template block="outport_declare">
  RTC::TimedString m_stop;
  RTC::OutPort<RTC::TimedString> m_stopOut;
  RTC::TimedString m_start_move;
  RTC::OutPort<RTC::TimedString> m_start_moveOut;
  // </rtc-template>

  // CORBA Port declaration
  // <rtc-template block="corbaport_declare">
  RTC::CorbaPort m_ManipulatorCommonInterface_CommonPort;
  RTC::CorbaPort m_ManipulatorCommonInterface_MiddlePort;
  // </rtc-template>

  // Consumer declaration
  // <rtc-template block="consumer_declare">
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Common> m_ManipulatorCommonInterface_Common;
  RTC::CorbaConsumer<JARA_ARM::ManipulatorCommonInterface_Middle> m_ManipulatorCommonInterface_Middle;
  // </rtc-template>

 private:
  // --- 変数定義 ---
  
  // 動作フェーズ管理 (0:Pick1, 1:Place, 2:Pick2, 3:Place)
  int phase; 
  
  // 待機用タイマーカウンタ
  int wait_timer;

  // 直前が危険状態だったかどうかを記録するフラグ
  bool was_danger;

  // 座標保持用
  JARA_ARM::JointPos Pick1Point; // Pick1
  JARA_ARM::JointPos PlacePoint; // Place
  JARA_ARM::JointPos Pick2Point; // Pick2

  // 内部関数: 現在のフェーズに応じた動作指令を送る
  void sendCurrentMotion();

};


extern "C"
{
  DLL_EXPORT void ManagerInit(RTC::Manager* manager);
};

#endif // MANAGER_H
