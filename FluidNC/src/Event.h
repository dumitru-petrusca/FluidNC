// Copyright (c) 2022 -  Mitch Bradley
// Use of this source code is governed by a GPLv3 license that can be found in the LICENSE file.

#pragma once

// Objects derived from the Event base class are placed in the event queue.
// Protocol dequeues them and calls their run methods.
class Event {
public:
    Event(char const* name) : _name(name) {}
    virtual void run(void* arg) = 0;

    char const* _name;
};

class NoArgEvent : public Event {
    void (*_function)() = nullptr;

public:
    NoArgEvent(char const* name, void (*function)()) : Event(name), _function(function) {}
    void run(void* arg) override {
        if (_function) {
            _function();
        }
    }
};

class ArgEvent : public Event {
    void (*_function)(void*) = nullptr;

public:
    ArgEvent(char const* name, void (*function)(void*)) : Event(name), _function(function) {}
    void run(void* arg) override {
        if (_function) {
            _function(arg);
        }
    }
};
