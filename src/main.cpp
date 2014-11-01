
/***************************************************************************
 *  main.cpp - Hybris Monitor GUI main
 *
 *  Created: Fri Oct 31 09:54:51 2014
 *  Copyright  2008-2014  Tim Niemueller [www.niemueller.de]
 *
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

/** This is the main program of the Hybris Monitor GUI.
 */
int
main(int argc, char **argv) {
  Gtk::Main gtk_main(argc, argv);
#ifdef HAVE_GCONFMM
  Gnome::Conf::init();
#endif

  int flags = ros::init_options::NoSigintHandler | ros::init_options::AnonymousName;
  ros::init(argc, argv, "hybris_monitor", flags);
  ros::AsyncSpinner spinner(1);
  spinner.start();

  try {
    Glib::RefPtr<Gtk::Builder> builder =
      Gtk::Builder::create_from_file(RESDIR"/monitor.ui");

    HybrisMonitorGtkWindow *window = NULL;
    builder->get_widget_derived("wnd_monitor", window);

    Gtk::Main::run(*window);

    delete window;
  } catch (Gtk::BuilderError &e) {
    printf("Failed to instantiate window: %s\n", e.what().c_str());
  }

  spinner.stop();
  ros::shutdown();

  return 0;
}
