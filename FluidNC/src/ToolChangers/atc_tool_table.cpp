// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "atc_tool_table.h"
#include "../Machine/MachineConfig.h"
#include "Settings.h"
#include <cstdio>
#include <iostream>
#include <nvs.h>

/*
  safe_z_mm: Set this to the mpos height you want the Z to travel around when tool changing. It is typically near the top so the longest tool can clear the work.
  change_mpos_mm: This is where the machine will go for the manual tool change. 

  ets_mpos_mm: The X and Y location are the XY center of the toolsetter. The Z is the lowest the Z should go before we fail due to missing bit.

  How do you tell FNC you already have a tool number installed before starting a job.

  How do you tell it you want to install a tool before a job.

  M6T0 from T<anything> -- Resets the offsets for a new job

  M6T<not 0> From T0 --  Moves to the change location and does nothing else. Assumes work zero needs to be set
  
  M6T<not 0> to T<anything> first time 
     -- Determines the TS offset
     -- Goes to toolchange location
     -- Set TLO
     -- Returns to position before command

  M6T<not 0> to T<anything> after first time
     -- Goes to toolchange location
     -- Set TLO
     -- Returns to position before command

  Posible New Persistant values (might want a save_ATC_values: config item. default false)
     -- TLO
     -- Tool number


tool_changer:
  safe_z_mpos_mm: -1.000000
  probe_seek_rate_mm_per_min: 800.000000
  probe_feed_rate_mm_per_min: 80.000000
  change_mpos_mm: 80.000 0.000 -1.000
  ets_mpos_mm: 5.000 -17.000 -40.000
*/

namespace ATCs {

    char* toolName(const char* prefix, int number) {
        int   n    = strlen(prefix);
        char* name = static_cast<char*>(malloc(n + 2 + 1));
        strcpy(name, prefix);
        itoa(number, name + n, 10);
        return name;
    }

    void ToolTable_ATC::init() {
        log_info("ATC:" << name());
        new ToolsCommand(this);

        for (int number = 0; number < MAX_TOOLS; ++number) {
            tool_keys[number]     = toolName("Tool", number);
            tools[number]._offset = 0;
            strcpy(tools[number]._name, "--");
            if (number != 0) {
                size_t len;
                nvs.get_blob(tool_keys[number], &tools[number], &len);
                new ToolCommand(number, toolName("T", number), tool_keys[number], this);
            }
        }
    }

    void ToolTable_ATC::probe_notification() {}

    bool ToolTable_ATC::tool_change(tool_t new_tool, bool pre_select, bool set_tool) {
        bool spindle_was_on = false;  // used to restore the spindle state
        bool inch_mode      = false;  // allows use to restore inch mode if req'd

        protocol_buffer_synchronize();  // wait for all motion to complete
        _macro.erase();                 // clear previous gcode

        // M61 - The ATC does nothing.
        // M6 T0 - is used to reset this ATC and allow us to start a new job
        if (set_tool || new_tool == 0) {
            _prev_tool = new_tool;
            if (new_tool == 0) {
                reset();  // clear TLO
            }
            _macro.run(nullptr);
            return true;
        }

        inch_mode = (gc_state.modal.units == Units::Inches);
        if (inch_mode) {
            _macro.addf("G21");
        }

        try {
            // turn off the spindle
            if (gc_state.modal.spindle != SpindleState::Disable) {
                spindle_was_on = true;
                _macro.addf("M5");
            }

            // compensate for the tool length (tool 1 is the reference, its offset is 0)
            _macro.addf("G43.1 Z%f", tools[new_tool]._offset);

            // wait for manual tool change
            _macro.addf("G4P0 0.1");
            _macro.addf("(MSG: Install tool #%d then resume to continue)", new_tool);
            _macro.addf("M0");

            if (spindle_was_on) {
                _macro.addf("M3");  // spindle should handle spinup delay
            }
            if (inch_mode) {
                _macro.addf("G20");
            }
            _prev_tool = new_tool;
            _macro.run(nullptr);
            return true;
        } catch (...) { log_info("Exception caught"); }

        return false;
    }

    void ToolTable_ATC::reset() {
        _is_OK     = true;
        _prev_tool = gc_state.selected_tool;     // Double check this
        _macro.addf("G43.1 Z0");                 // reset the TLO to 0
        _macro.addf("(MSG: TLO Z reset to 0)");  //
    }

    void ToolTable_ATC::set_tool(int number, char* value) {
        std::strcpy(tools[number]._name, std::strtok(value, ","));
        tools[number]._offset = std::stof(std::strtok(nullptr, ","));
        if (gc_state.modal.units == Units::Inches) {
            tools[number]._offset *= MM_PER_INCH;
        }
        nvs.set_blob(tool_keys[number], &tools[number], sizeof(Tool));
    }

    std::string ToolTable_ATC::tool_string(int number) {
        char buf[64];
        sprintf(buf, "%d,%s,%.3f", number, tools[number]._name, tools[number]._offset);
        return std::string(buf);
    }

    Error ToolCommand::action(const char* value, AuthenticationLevel auth_level, Channel& response) {
        if (value != NULL) {
            _atc->set_tool(_number, const_cast<char*>(value));
        }
        log_stream(response, "[TOOL:" << _atc->tool_string(_number) << "]");
        return Error::Ok;
    }

    Error ToolsCommand::action(const char* value, AuthenticationLevel auth_level, Channel& response) {
        std::ostringstream msg;
        for (int number = 1; number < MAX_TOOLS; ++number) {
            if (number != 1) {
                msg << ";";
            }
            msg << _atc->tool_string(number);
        }
        log_stream(response, "[TOOLS:" << msg.str() << "]");
        return Error::Ok;
    }

    namespace {
        ATCFactory::InstanceBuilder<ToolTable_ATC> registration("atc_tool_table");
    }
}
