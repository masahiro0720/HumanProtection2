// -*- C++ -*-
/*!
 * @file  HumanProtection.cpp
 * @brief Human Protection RT Component
 * @date $Date$
 *
 * $Id$
 */

#include "HumanProtection.h"
#include <chrono> // 時間計測用

// Module specification
static const char* humanprotection_spec[] =
  {
    "implementation_id", "HumanProtection",
    "type_name",         "HumanProtection",
    "description",       "Human Protection RT Component ",
    "version",           "1.0.0",
    "vendor",            "Robot System Design Laboratory, Meijo Univ.",
    "category",          "Controller",
    "activity_type",     "PERIODIC",
    "kind",              "DataFlowComponent",
    "max_instance",      "1",
    "language",          "C++",
    "lang_type",         "compile",
    "conf.default.judge_parameter", "1500",
    "conf.__widget__.judge_parameter", "text",
    "conf.__type__.judge_parameter", "double",
    ""
  };

// 時間計測用の変数（簡易実装）
static std::chrono::system_clock::time_point danger_start_time;
static bool is_danger_counting = false;

// 【修正点】1.0秒から0.5秒に変更
static const double DANGER_THRESHOLD_TIME = 0.5; 

HumanProtection::HumanProtection(RTC::Manager* manager)
  : RTC::DataFlowComponentBase(manager),
    m_human_poseIn("HumanPose", m_human_pose),
    m_stop_comOut("StopCommand", m_stop_com)
{
}

HumanProtection::~HumanProtection()
{
}

RTC::ReturnCode_t HumanProtection::onInitialize()
{
  addInPort("HumanPose", m_human_poseIn);
  addOutPort("StopCommand", m_stop_comOut);
  bindParameter("judge_parameter", m_judge_parameter, "1500");
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HumanProtection::onActivated(RTC::UniqueId ec_id)
{
  // 起動時にタイマーリセット
  is_danger_counting = false;
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HumanProtection::onDeactivated(RTC::UniqueId ec_id)
{
  return RTC::RTC_OK;
}

RTC::ReturnCode_t HumanProtection::onExecute(RTC::UniqueId ec_id)
{
  // データが来ているかチェック
  if (m_human_poseIn.isNew())
  {
    m_human_poseIn.read();
    
    // std::cout << "z:= " << m_human_pose.pose_q.p3D.z << std::endl;

    bool current_danger = false;

    // Detectionから (0,0,0) が来た場合は「手がない」＝「安全」
    if (m_human_pose.pose_q.p3D.x == 0 && m_human_pose.pose_q.p3D.y == 0 && m_human_pose.pose_q.p3D.z == 0)
    {
      current_danger = false;
    }
    // 手の距離(z)が0より大きく、かつ判定パラメータ以下なら「危険」
    else if ( 0 < m_human_pose.pose_q.p3D.z && m_human_pose.pose_q.p3D.z <= m_judge_parameter)
    {
      current_danger = true;
    }
    else
    {
      current_danger = false;
    }

    // ==========================================
    // 【改良】継続検知ロジック (0.5秒)
    // ==========================================
    if (current_danger)
    {
        // 危険状態が始まったばかりなら時刻を記録
        if (!is_danger_counting) {
            danger_start_time = std::chrono::system_clock::now();
            is_danger_counting = true;
            m_stop_com.data = 0; // まだ停止しない
        }
        else {
            // 経過時間を計算
            std::chrono::duration<double> elapsed_seconds = std::chrono::system_clock::now() - danger_start_time;
            
            if (elapsed_seconds.count() >= DANGER_THRESHOLD_TIME) {
                // 指定時間以上経過したので本当に停止させる
                m_stop_com.data = 1;
                printf("DANGER DETECTED (> 0.5s)! Sending STOP.\r\n");
            } else {
                // まだ指定時間経っていない
                m_stop_com.data = 0;
            }
        }
    }
    else
    {
        // 安全ならカウンターリセット
        is_danger_counting = false;
        m_stop_com.data = 0; // 安全
        // printf("Safe.\r\n");
    }
    
    // コマンド出力
    m_stop_comOut.write();
  }
  
  return RTC::RTC_OK;
}

extern "C"
{
  void HumanProtectionInit(RTC::Manager* manager)
  {
    coil::Properties profile(humanprotection_spec);
    manager->registerFactory(profile,
                             RTC::Create<HumanProtection>,
                             RTC::Delete<HumanProtection>);
  }
};
