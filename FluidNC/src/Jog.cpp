// Copyright (c) 2016 Sungeun K. Jeon for Gnea Research LLC
// Copyright (c) 2018 -	Bart Dring
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#include "Jog.h"

#include "Machine/MachineConfig.h"
#include "MotionControl.h"  // mc_linear
#include "Stepper.h"        // st_prep_buffer, st_wake_up
#include "Limits.h"         // constrainToSoftLimits()

// Sets up valid jog motion received from g-code parser, checks for soft-limits, and executes the jog.
// cancelledInflight will be set to true if was not added to parser due to a cancelJog.
Error jog_execute(plan_line_data_t* pl_data, parser_block_t* gc_block, bool* cancelledInflight) {
    config->_kinematics->constrain_jog(gc_block->values.xyz, pl_data, gc_state.position);

    // Initialize planner data struct for jogging motions.
    // NOTE: Spindle and coolant are allowed to fully function with overrides during a jog.
    pl_data->feed_rate             = gc_block->values.f;
    pl_data->motion.noFeedOverride = 1;
    pl_data->is_jog                = true;
    pl_data->line_number           = gc_block->values.n;

    if (!mc_linear(gc_block->values.xyz, pl_data, gc_state.position)) {
        return Error::JogCancelled;
    }

    // The motion will be initiated by the cycle start mechanism
    return Error::Ok;
}

/*
void jog_mpg() {
    float        feed = config->_axes->_axis[0]->_mpg->_max_rate_mm_per_min;  //TODO-dp
    static float target[MAX_N_AXIS];
    bool         worthTheJog = false;

    for (int i = 0; i < config->_axes->_numberAxis; i++) {
        target[i] = gc_state.position[i];
        MPG* mpg  = config->_axes->_axis[i]->_mpg;
        if (mpg != nullptr && !mpg->_locked) {
            int64_t dci = mpg->_encoder->getCountDelta();
            float   di  = dci * mpg->_mm_per_step;
            target[i] += di;
            worthTheJog = true;
        }
    }

    if (worthTheJog) {
        plan_line_data_t pl_data = {};
        memset(&pl_data, 0, sizeof(plan_line_data_t));  // Zero pl_data struct
        pl_data.spindle_speed         = gc_state.spindle_speed;
        pl_data.spindle               = gc_state.modal.spindle;
        pl_data.coolant               = gc_state.modal.coolant;
        pl_data.motion.noFeedOverride = 1;
        pl_data.is_jog                = true;
        pl_data.feed_rate             = feed;
        if (mc_linear(target, &pl_data, gc_state.position)) {
            copyAxes(gc_state.position, target);
        }
    }
}
*/
