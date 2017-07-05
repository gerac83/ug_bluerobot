// Copyright (c) CloPeMa, EU-FP7 - All Rights Reserved
//
// Author:      Libor Wagner <wagnelib@cmp.felk.cvut.cz>
// Institute:   Czech Technical University in Prague
// Created on:  Feb 19, 2014

#include "clopema_libs/ui_utils.h"
#include <iostream>
#include <algorithm>
#include <cctype>

// ====================== Prototypes of helper functions ======================

/** \brief Get index of first  upper character in the string. */
std::size_t get_first_upper(std::string str);

// ====================== Implementation of public functions ==================
char ask(std::string question, std::string options)
{
    bool ok = false;
    bool has_default = false;
    std::size_t found;
    char reply, def;

    std::string options_lower = options;
    std::transform(options_lower.begin(), options_lower.end(), options_lower.begin(), ::tolower);

    found = get_first_upper(options);
    if (found != std::string::npos) {
        has_default = true;
        def = options_lower.at(found);
    }

    while (!ok) {
        std::cout << question << " [" << options << "]: ";
        std::string line;
        std::getline (std::cin, line);

        if (line.size() <= 0) {
            if (has_default) {
                reply = def;
                ok = true;
            }
        } else {

            found = options_lower.find(line);
            if (found != std::string::npos) {
                reply = line.at(0);
                ok = true;
            }
        }
    }

    return reply;
}

// ===================== Implementation of helper functions ===================

std::size_t get_first_upper(std::string str)
{
    for (std::size_t i = 0; i < str.size(); i++) {
        if (isupper(str.at(i)) != 0) {
            return i;
        }
    }
    return std::string::npos;
}
