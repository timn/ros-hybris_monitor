
/***************************************************************************
 *  hybris_monitor.h - Hybris Monitor GUI
 *
 *  Created: Mon Nov 03 13:35:34 2008
 *  Copyright  2014  Tim Niemueller [www.niemueller.de]
 ****************************************************************************/

/*  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  Read the full text in the LICENSE.GPL file in the doc directory.
 */

#ifndef __SRC_HYBRIS_MONITOR_H_
#define __SRC_HYBRIS_MONITOR_H_


#include <gtkmm.h>
#ifdef HAVE_GCONFMM
#  include <gconfmm.h>
#  define GCONF_PREFIX "/apps/hybris_monitor"
#endif

#include <ros/ros.h>
#include <actionlib/client/action_client.h>

#include <continual_planning_executive/ContinualPlanningStatus.h>
#include <fawkes_msgs/SkillStatus.h>
#include <std_msgs/String.h>

#include <boost/thread/mutex.hpp>
#include <queue>

class HybrisMonitorGtkWindow : public Gtk::Window
{
 public:  
  HybrisMonitorGtkWindow(BaseObjectType* cobject,
			 const Glib::RefPtr<Gtk::Builder> &builder);
  ~HybrisMonitorGtkWindow();

 private:

  void on_config_changed();
  //void ros_skiller_graphmsg_cb(const skiller::Graph::ConstPtr &msg);
  void planner_status_cb(const continual_planning_executive::ContinualPlanningStatus &status);
  void on_planner_status_cb();
  void skiller_status_cb(const fawkes_msgs::SkillStatus &status);
  void on_skiller_status_cb();
  void agent_info_cb(const std_msgs::String &info);
  void on_agent_info_cb();
  std::string get_action_description(std::string action);

  class PlanStepRecord : public Gtk::TreeModelColumnRecord
  {
  public:
    PlanStepRecord() {
      add(step);
      add(foreground);
      add(background);
    }
    Gtk::TreeModelColumn<Glib::ustring> step;      /**< The plan step description */
    Gtk::TreeModelColumn<Gdk::Color>    foreground;
    Gtk::TreeModelColumn<Gdk::Color>    background;
  };


 private:
  Gtk::Label             *lab_agent_now_;
  Gtk::Label             *lab_agent_before_;
  Gtk::TreeView          *trv_plan_;
  Gtk::ScrolledWindow    *window_plan_;
  Gtk::Label             *lab_skill_;

  Glib::RefPtr<Gtk::ListStore> plan_list_;
  PlanStepRecord               plan_record_;

#ifdef HAVE_GCONFMM
  Glib::RefPtr<Gnome::Conf::Client> gconf_;
#endif

  Glib::Dispatcher             dsp_planner_status_;
  Glib::Dispatcher             dsp_skiller_status_;
  Glib::Dispatcher             dsp_agent_info_;

  ros::NodeHandle ros_nh_;
  ros::Subscriber sub_planner_status_;
  ros::Subscriber sub_skiller_status_;
  ros::Subscriber sub_agent_info_;

  boost::mutex msgmtx_planner_status_;
  std::queue<continual_planning_executive::ContinualPlanningStatus> msgq_planner_status_;

  boost::mutex msgmtx_skiller_status_;
  std::queue<fawkes_msgs::SkillStatus> msgq_skiller_status_;

  boost::mutex msgmtx_agent_info_;
  std::queue<std_msgs::String> msgq_agent_info_;
};

#endif
