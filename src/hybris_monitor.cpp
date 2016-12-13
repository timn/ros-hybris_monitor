
/***************************************************************************
 *  hybris_monitor.cpp - Hybris Monitor GUI
 *
 *  Created: Mon Nov 03 13:37:33 2008
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

#include "hybris_monitor.h"

#include <boost/algorithm/string.hpp>

#include <cstring>
#include <string>

#include <gtkmm/adjustment.h>

using namespace ros;

/** @class HybrisMonitorGtkWindow "hybris_monitor.h"
 * Hybris Monitor GUI main window.
 * @author Tim Niemueller
 */

/** Constructor.
 * @param cobject C base object
 * @param builder instance to retrieve widgets from
 */
HybrisMonitorGtkWindow::HybrisMonitorGtkWindow(BaseObjectType* cobject,
					       const Glib::RefPtr<Gtk::Builder> &builder)
  : Gtk::Window(cobject), ros_nh_()
{
  set_wmclass("Hybris Robot Monitor", "Hybris Robot Monitor");

#ifdef HAVE_GCONFMM
  gconf_ = Gnome::Conf::Client::get_default_client();
  gconf_->add_dir(GCONF_PREFIX);
#endif

  builder->get_widget("lab_agent_now", lab_agent_now_);
  builder->get_widget("lab_agent_before", lab_agent_before_);
  builder->get_widget("lab_skill", lab_skill_);
  builder->get_widget("trv_plan", trv_plan_);
  builder->get_widget("window_plan", window_plan_);
  builder->get_widget("lab_plan_status", lab_plan_status_);

  plan_list_ = Gtk::ListStore::create(plan_record_);
  trv_plan_->set_model(plan_list_);
  trv_plan_->append_column("Step", plan_record_.step);
  trv_plan_->get_selection()->set_mode(Gtk::SELECTION_NONE);

  Glib::ListHandle<Gtk::TreeViewColumn *> columns = trv_plan_->get_columns();
  for (Glib::ListHandle<Gtk::TreeViewColumn *>::iterator c = columns.begin();
       c != columns.end();
       ++c)
  {
#if GTK_VERSION_GE(3,0)
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell();
#else
    Gtk::CellRenderer *cell_renderer = (*c)->get_first_cell_renderer();
#endif
    Gtk::CellRendererText *text_renderer =
      dynamic_cast<Gtk::CellRendererText *>(cell_renderer);
    if ( text_renderer ) {
      Pango::AttrList  attr_list;
      Pango::Attribute scale_attr = Pango::Attribute::create_attr_scale(1.25);
      attr_list.insert(scale_attr);
      text_renderer->property_attributes().set_value(attr_list);
#ifdef GLIBMM_PROPERTIES_ENABLED
      (*c)->add_attribute(text_renderer->property_background_gdk(), plan_record_.background);
      (*c)->add_attribute(text_renderer->property_foreground_gdk(), plan_record_.foreground);
#else
      (*c)->add_attribute(*text_renderer, "background-gdk", plan_record_.background);
      (*c)->add_attribute(*text_renderer, "foreground-gdk", plan_record_.background);
#endif
    }
  }

  //graph_changed.connect(sigc::mem_fun(*this, &HybrisMonitorGtkWindow::on_graph_changed));

  dsp_planner_status_.connect(sigc::mem_fun(*this, &HybrisMonitorGtkWindow::on_planner_status_cb));
  sub_planner_status_ = ros_nh_.subscribe("continual_planning_status", 10,
					  &HybrisMonitorGtkWindow::planner_status_cb, this);

  dsp_skiller_status_.connect(sigc::mem_fun(*this, &HybrisMonitorGtkWindow::on_skiller_status_cb));
  sub_skiller_status_ = ros_nh_.subscribe("skiller_status", 10,
					  &HybrisMonitorGtkWindow::skiller_status_cb, this);

  dsp_agent_info_.connect(sigc::mem_fun(*this, &HybrisMonitorGtkWindow::on_agent_info_cb));
  sub_agent_info_ = ros_nh_.subscribe("agent_info", 10,
					  &HybrisMonitorGtkWindow::agent_info_cb, this);

#ifdef HAVE_GCONFMM
  gconf_->signal_value_changed().connect(
    sigc::hide(sigc::hide(sigc::mem_fun(*this, &HybrisMonitorGtkWindow::on_config_changed))));
  on_config_changed();
#endif
}


/** Destructor. */
HybrisMonitorGtkWindow::~HybrisMonitorGtkWindow()
{
#ifdef HAVE_GCONFMM
  gconf_->remove_dir(GCONF_PREFIX);
#endif
}


void
HybrisMonitorGtkWindow::on_config_changed()
{
#ifdef HAVE_GCONFMM
  /*
  Gnome::Conf::SListHandle_ValueString l(__gconf->get_string_list(GCONF_PREFIX"/command_history"));

  __sks_list->clear();
  for (Gnome::Conf::SListHandle_ValueString::const_iterator i = l.begin(); i != l.end(); ++i) {
    Gtk::TreeModel::Row row  = *__sks_list->append();
    row[__sks_record.skillstring] = *i;    
  }
  */
#endif
}


std::string
HybrisMonitorGtkWindow::get_action_description(std::string action)
{
  std::string::size_type start_pos = action.find("(");
  std::string::size_type end_pos   = action.rfind(")");

  if (start_pos != std::string::npos && end_pos != std::string::npos) {
    return action.substr(start_pos, end_pos - start_pos + 1);
  } else {
    return action;
  }
}

void
HybrisMonitorGtkWindow::on_planner_status_cb()
{
  continual_planning_msgs::ContinualPlanningStatus status;

  {
    boost::mutex::scoped_lock lock(msgmtx_planner_status_);
    if (msgq_planner_status_.empty()) {
      return;
    }
    status = msgq_planner_status_.front();
    msgq_planner_status_.pop();
  }

  // Some weird convention to keep the "before" state
  if (status.description == "-")  return;

  switch(status.component) {
  case continual_planning_msgs::ContinualPlanningStatus::PLANNING:
    {
      std::vector<std::string> strs, tmp;
      boost::split(tmp, status.description, boost::is_any_of("\n"));
      for (size_t i = 0; i < tmp.size(); ++i) {
	if (tmp[i] != "") {
	  strs.push_back(tmp[i]);
	}
      }
      plan_list_->clear();
      for (size_t i = 0; i < strs.size(); ++i) {
	Gtk::TreeModel::Row row;
	row = *(plan_list_->append());
	row[plan_record_.step] = strs[i];
      }

      if (strs.empty()) {
	      lab_plan_status_->set_text("");
      } else {
	      std::string status_string = "Plan size: " + std::to_string(strs.size());
	      lab_plan_status_->set_text(status_string);
      }
    }
    break;

  case continual_planning_msgs::ContinualPlanningStatus::CURRENT_PLAN:
    {
      std::vector<std::string> strs, tmp;
      boost::split(tmp, status.description, boost::is_any_of("\n"));
      for (size_t i = 0; i < tmp.size(); ++i) {
        if (tmp[i] != "") {
          strs.push_back(tmp[i]);
        }
      }
      Gtk::TreeModel::Children children = plan_list_->children();

      ssize_t active = -1;
      if (children.size() >= strs.size()) {
	active = children.size() - strs.size();
      }

      ssize_t i = 0;
      for (Gtk::TreeModel::Children::iterator c = children.begin(); c != children.end(); ++c) {
	Gdk::Color color;
	if (i == active) {
	  //trv_plan_->set_cursor(plan_list_->get_path(c));
	  color.set_rgb_p(0.8, 0.8, 1.0);
	} else {
	  color.set_rgb_p(1.0, 1.0, 1.0);
	}
	(*c)[plan_record_.background] = color;
	++i;
      }
    }
    break;

      /*
  case continual_planning_msgs::ContinualPlanningStatus::EXECUTION:
    {
      std::string active_action = get_action_description(status.description);

      Gtk::TreeModel::Children children = plan_list_->children();
      for (Gtk::TreeModel::Children::iterator c = children.begin(); c != children.end(); ++c) {
	Gdk::Color color;
	if (get_action_description((Glib::ustring)(*c)[plan_record_.step]) == active_action) {
	  //trv_plan_->set_cursor(plan_list_->get_path(c));
	  color.set_rgb_p(0.8, 0.8, 1.0);
	} else {
	  color.set_rgb_p(1.0, 1.0, 1.0);
	}
	(*c)[plan_record_.background] = color;
      }
    }
    break;
      */

  default: // ignored
    break;
  } 

  Glib::RefPtr<Gtk::Adjustment> adjustment = window_plan_->get_vadjustment();
  adjustment->set_value(adjustment->get_upper());
}


void
HybrisMonitorGtkWindow::planner_status_cb(const continual_planning_msgs::ContinualPlanningStatus &status)
{
  {
    boost::mutex::scoped_lock lock(msgmtx_planner_status_);
    msgq_planner_status_.push(status);
  }
  dsp_planner_status_();
}


void
HybrisMonitorGtkWindow::on_skiller_status_cb()
{
  fawkes_msgs::SkillStatus status;

  {
    boost::mutex::scoped_lock lock(msgmtx_skiller_status_);
    if (msgq_skiller_status_.empty()) {
      return;
    }
    status = msgq_skiller_status_.front();
    msgq_skiller_status_.pop();
  }


  switch (status.status) {
  case fawkes_msgs::SkillStatus::S_FINAL:
    lab_skill_->set_markup("<span foreground=\"#008800\">" + status.skill_string + "</span>");
    break;

  case fawkes_msgs::SkillStatus::S_FAILED:
    lab_skill_->set_markup("<span foreground=\"#880000\">" + status.error + "</span>");
    break;

  case fawkes_msgs::SkillStatus::S_RUNNING:
    lab_skill_->set_text(status.skill_string);
    break;

  default: // INACTIVE
    lab_skill_->set_text("");
    break;
  }
  
}

void
HybrisMonitorGtkWindow::skiller_status_cb(const fawkes_msgs::SkillStatus &status)
{
  {
    boost::mutex::scoped_lock lock(msgmtx_skiller_status_);
    msgq_skiller_status_.push(status);
  }
  dsp_skiller_status_();
}


void
HybrisMonitorGtkWindow::on_agent_info_cb()
{
  std_msgs::String info;

  {
    boost::mutex::scoped_lock lock(msgmtx_agent_info_);
    if (msgq_agent_info_.empty()) {
      return;
    }
    info = msgq_agent_info_.front();
    msgq_agent_info_.pop();
  }

  if (info.data == "--reset--") {
    lab_agent_now_->set_text("");
    lab_agent_before_->set_text("");
    lab_skill_->set_text("");
    plan_list_->clear();
    lab_plan_status_->set_text("");
  } else {
    lab_agent_before_->set_text(lab_agent_now_->get_text());
    lab_agent_now_->set_text(info.data);
  }
}

void
HybrisMonitorGtkWindow::agent_info_cb(const std_msgs::String &info)
{
  {
    boost::mutex::scoped_lock lock(msgmtx_agent_info_);
    msgq_agent_info_.push(info);
  }
  dsp_agent_info_();
}

