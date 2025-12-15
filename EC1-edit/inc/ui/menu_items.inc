Menu main_menu_settings = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "      Settings      ",
.parent = nullptr,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = nullptr,
};
    Menu settings_cycles_run = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "                    ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_cycles_run,
    };
    main_menu_settings.child = &settings_cycles_run;

    Menu settings_set_time = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "      Set Time      ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = edit_system_time,
    };

    Menu settings_dispense_time = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "   Dispense Time    ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = edit_dispense_time,
    };

    Menu settings_num_of_rinses = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "    # of Rinses     ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = edit_num_rinses,
    };

    Menu settings_cycle_mode = {
    .tag = 0,
    .line1 = "     Cycle Mode     ",
    .line2 = "                    ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = nullptr,
    };

    Menu settings_version = {
    .tag = 0,
    .line1 = "  Software Version  ",
    .line2 = VERSION_STRING,
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = nullptr,
    };

    Menu settings_exit = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "            EXIT?   ",
    .parent = &main_menu_settings,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = exit_menu,
    };

    settings_cycles_run.up = nullptr;
    settings_set_time.up = &settings_cycles_run;
    settings_dispense_time.up = &settings_set_time;
    settings_num_of_rinses.up = &settings_dispense_time;
    settings_cycle_mode.up = &settings_num_of_rinses;
    settings_version.up = &settings_cycle_mode;
    settings_exit.up = &settings_version;

    settings_cycles_run.down = &settings_set_time;
    settings_set_time.down = &settings_dispense_time;
    settings_dispense_time.down = &settings_num_of_rinses;
    settings_num_of_rinses.down = &settings_cycle_mode;
    settings_cycle_mode.down = &settings_version;
    settings_version.down = &settings_exit;
    settings_exit.down = nullptr;

Menu main_menu_first_time = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "      1st Time      ",
.parent = nullptr,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = nullptr,
};

    Menu first_time_purge_and_rinse = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "    Run 1st Time    ",
    .parent = &main_menu_first_time,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = prime,
    };
    main_menu_first_time.child = &first_time_purge_and_rinse;

    Menu first_time_exit = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "            EXIT?   ",
    .parent = &main_menu_first_time,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = exit_menu,
    };

    first_time_purge_and_rinse.up = nullptr;
    first_time_exit.up = &first_time_purge_and_rinse;

    first_time_purge_and_rinse.down = &first_time_exit;
    first_time_exit.down = nullptr;

Menu main_menu_manual_mode = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "     Manual Mode    ",
.parent = nullptr,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = nullptr,
};

    Menu manual_mode_rotary_valve_and_syringe = {
    .tag = 0,
    .line1 = "  Rotary Valve   ",
    .line2 = "   & Syringe     ",
    .parent = &main_menu_manual_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = rot_syringe_test,
    };
    main_menu_manual_mode.child = &manual_mode_rotary_valve_and_syringe;

    Menu manual_mode_flush = {
    .tag = 0,
    .line1 = "                 ",
    .line2 = "      Rinse      ",
    .parent = &main_menu_manual_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = manual_rinse,
    };

    Menu manual_mode_rehome = {
    .tag = 0,
    .line1 = "                 ",
    .line2 = "     Rehome      ",
    .parent = &main_menu_manual_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = rehome,
    };

    Menu manual_mode_run_non_stop = {
    .tag = 0,
    .line1 = "                 ",
    .line2 = "  Run non-stop   ",
    .parent = &main_menu_manual_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = cycle_continuously,
    };

    Menu manual_mode_exit = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "            EXIT?   ",
    .parent = &main_menu_manual_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = exit_menu,
    };

    manual_mode_rotary_valve_and_syringe.up = nullptr;
    manual_mode_flush.up = &manual_mode_rotary_valve_and_syringe;
    manual_mode_rehome.up = &manual_mode_flush;

    #ifdef QC
        manual_mode_run_non_stop.up = &manual_mode_rehome;
        manual_mode_exit.up = &manual_mode_run_non_stop;
    #else
        manual_mode_exit.up = &manual_mode_rehome;
    #endif

    manual_mode_rotary_valve_and_syringe.down = &manual_mode_flush;
    manual_mode_flush.down = &manual_mode_rehome;
    
    #ifdef QC
        manual_mode_rehome.down = &manual_mode_run_non_stop;
        manual_mode_run_non_stop.down = &manual_mode_exit;
    #else
        manual_mode_rehome.down = &manual_mode_exit;
    #endif

    manual_mode_exit.down = nullptr;

Menu main_menu_errors = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "       Alarms       ",
.parent = nullptr,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = nullptr,
};
    Menu errors_overflow = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "      Overflow      ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };
    main_menu_errors.child = &errors_overflow;

    Menu errors_rotary = {
    .tag = 1,
    .line1 = "                    ",
    .line2 = "   Rotary Homing    ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };

    Menu errors_syringe = {
    .tag = 2,
    .line1 = "                    ",
    .line2 = "   Syringe Homing   ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };

    Menu errors_high_temp = {
    .tag = 3,
    .line1 = "                    ",
    .line2 = "  High Temperature  ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };

    Menu errors_low_temp = {
    .tag = 4,
    .line1 = "                    ",
    .line2 = "  Low Temperature   ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };

    Menu errors_battery = {
    .tag = 5,
    .line1 = "                    ",
    .line2 = "    Low Battery     ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = reset_warning,
    };

    Menu errors_exit = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "            EXIT?   ",
    .parent = &main_menu_errors,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = exit_menu,
    };

    errors_overflow.up = nullptr;
    errors_rotary.up = &errors_overflow;
    errors_syringe.up = &errors_rotary;
    errors_high_temp.up = &errors_syringe;
    errors_low_temp.up = &errors_high_temp;
    errors_battery.up = &errors_low_temp;
    errors_exit.up = &errors_battery;

    errors_overflow.down = &errors_rotary;
    errors_rotary.down = &errors_syringe;
    errors_syringe.down = &errors_high_temp;
    errors_high_temp.down = &errors_low_temp;
    errors_low_temp.down = &errors_battery;
    errors_battery.down = &errors_exit;
    errors_exit.down = nullptr;

main_menu_settings.up = nullptr;
main_menu_first_time.up = &main_menu_settings;
main_menu_manual_mode.up = &main_menu_first_time;
main_menu_errors.up = &main_menu_manual_mode;

main_menu_settings.down = &main_menu_first_time;
main_menu_first_time.down = &main_menu_manual_mode;
main_menu_manual_mode.down = &main_menu_errors;
main_menu_errors.down = nullptr;

Menu hidden_menu_cycle_mode = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "     Cycle Mode     ",
.parent = &main_menu_settings,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = nullptr
};

    Menu hidden_menu_drains = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "  Default (drains)  ",
    .parent = &hidden_menu_cycle_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = set_cycle_mode
    };
    hidden_menu_cycle_mode.child = &hidden_menu_drains;

    Menu hidden_menu_wastewater = {
    .tag = 1,
    .line1 = "                    ",
    .line2 = " Wastewater (mild)  ",
    .parent = &hidden_menu_cycle_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = set_cycle_mode
    };

    Menu hidden_menu_anaerobic = {
    .tag = 2,
    .line1 = "                    ",
    .line2 = " Anaerobic Digester ",
    .parent = &hidden_menu_cycle_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = set_cycle_mode
    };

    Menu hidden_menu_cycle_exit = {
    .tag = 0,
    .line1 = "                    ",
    .line2 = "            EXIT?   ",
    .parent = &hidden_menu_cycle_mode,
    .child = nullptr,
    .up = nullptr,
    .down = nullptr,
    .callback = exit_menu
    };

    hidden_menu_drains.up = nullptr;
    hidden_menu_wastewater.up = &hidden_menu_drains;
    hidden_menu_anaerobic.up = &hidden_menu_wastewater;
    hidden_menu_cycle_exit.up = &hidden_menu_anaerobic;

    hidden_menu_drains.down = &hidden_menu_wastewater;
    hidden_menu_wastewater.down = &hidden_menu_anaerobic;
    hidden_menu_anaerobic.down = &hidden_menu_cycle_exit;
    hidden_menu_cycle_exit.down = nullptr;

Menu hidden_menu_cycle_number = {
.tag = 0,
.line1 = "       SELECT       ",
.line2 = "    # of Cycles     ",
.parent = &main_menu_settings,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = edit_cycle_count
};

Menu hidden_menu_cycle_now = {
.tag = 0,
.line1 = "                    ",
.line2 = "  Start Cycle Now   ",
.parent = &main_menu_settings,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = cycle_now,
};

Menu hidden_menu_exit = {
.tag = 0,
.line1 = "                    ",
.line2 = "            EXIT?   ",
.parent = &main_menu_settings,
.child = nullptr,
.up = nullptr,
.down = nullptr,
.callback = exit_menu
};

hidden_menu_cycle_mode.up = nullptr;
hidden_menu_cycle_number.up = &hidden_menu_cycle_mode;
hidden_menu_cycle_now.up = &hidden_menu_cycle_number;
hidden_menu_exit.up = &hidden_menu_cycle_now;

hidden_menu_cycle_mode.down = &hidden_menu_cycle_number;
hidden_menu_cycle_number.down = &hidden_menu_cycle_now;
hidden_menu_cycle_now.down = &hidden_menu_exit;
hidden_menu_exit.down = nullptr;

const Menu* main_menu = &main_menu_settings;
const Menu* hidden_menu = &hidden_menu_cycle_mode;