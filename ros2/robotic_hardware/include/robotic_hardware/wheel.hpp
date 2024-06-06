#include <string>

class Wheel
{
public:
    std::string name = "";
    double pos = 0;
    double vel = 0;
    double cmd = 0;

    Wheel() = default;

    Wheel(const std::string &wheel_name)
    {
        setup(wheel_name);
    }

    void setup(const std::string &wheel_name)
    {
        name = wheel_name;
    }
};
