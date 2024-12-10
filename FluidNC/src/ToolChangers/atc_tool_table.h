// Copyright (c) 2024 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

#include <cfloat>
#include "Config.h"

#include "Configuration/Configurable.h"

#include "Channel.h"
#include "Module.h"
#include "Settings.h"
#include "atc.h"
#include "../Machine/Macros.h"

#define MAX_TOOLS 16

namespace ATCs {

    struct Tool {
        char  _name[32];
        float _offset;
    };

    class ToolTable_ATC : public ATC {
    public:
        char* tool_keys[MAX_TOOLS];
        Tool  tools[MAX_TOOLS];

        ToolTable_ATC(const char* name) : ATC(name) {}

        ToolTable_ATC(const ToolTable_ATC&)            = delete;
        ToolTable_ATC(ToolTable_ATC&&)                 = delete;
        ToolTable_ATC& operator=(const ToolTable_ATC&) = delete;
        ToolTable_ATC& operator=(ToolTable_ATC&&)      = delete;

        virtual ~ToolTable_ATC() = default;

    private:
        bool    _is_OK     = false;
        uint8_t _prev_tool = 0;  // TODO This could be a NV setting
        Macro   _macro;

        void reset();

    public:
        void        init() override;
        void        set_tool(int number, char* value);
        std::string tool_string(int number);
        void        probe_notification() override;
        bool        tool_change(tool_t value, bool pre_select, bool set_tool) override;
        void        validate() override {}
        void        group(Configuration::HandlerBase& handler) override {}
    };

    class ToolCommand : public Command {
    private:
        int            _number;
        ToolTable_ATC* _atc;

    public:
        ToolCommand(int n, char* name, char* fullName, ToolTable_ATC* atc) :
            Command(NULL, GRBLCMD, WG, name, fullName, anyState, true), _number(n), _atc(atc) {}
        Error action(const char* value, AuthenticationLevel auth_level, Channel& response);
    };

    class ToolsCommand : public Command {
    private:
        ToolTable_ATC* _atc;

    public:
        ToolsCommand(ToolTable_ATC* atc) : Command(NULL, GRBLCMD, WG, "TT", "Report Tools", anyState, true), _atc(atc) {}
        Error action(const char* value, AuthenticationLevel auth_level, Channel& response);
    };
}
