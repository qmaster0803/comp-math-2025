#pragma once
#include "brick.h"
#include <vector>

typedef std::vector<double> system_state_t;

class System
{
public:
    System(std::vector<Brick_size> sizes);
    std::vector<Brick> bricks;
    void operator()(const system_state_t &x, system_state_t &dxdt, double t);
    void observer(const system_state_t &x, double t) const;
    system_state_t get_state_as_array() const;
};


