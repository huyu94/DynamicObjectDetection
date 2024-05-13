#pragma once

#include <std_msgs/ColorRGBA.h>



class Color : public std_msgs::ColorRGBA
{
public:
    Color() : std_msgs::ColorRGBA(){}
    Color(double red, double green, double blue) : Color(red, green, blue, 1.0) {}
    Color(double red, double green, double blue, double alpha) : Color()
    {
        r = red;
        g = green;
        b = blue;
        a = alpha;
    }

    static const Color White() { return Color(1.0, 1.0, 1.0); }
    static const Color Black() { return Color(0.0, 0.0, 0.0); } // 
    static const Color Gray() { return Color(0.5, 0.5, 0.5); } // 优化失败轨迹
    static const Color Red() { return Color(1.0, 0.0, 0.0); } // 优化成功轨迹
    static const Color Green() { return Color(0.0, 0.7, 0.0); }
    static const Color Blue() { return Color(0.0, 0.0, 1.0); } // Kino traj color 
    static const Color SteelBlue() { return Color(0.4, 0.7, 1.0); }
    static const Color Yellow() { return Color(1.0, 1.0, 0.0); }
    static const Color Orange() { return Color(1.0, 0.5, 0.0); }
    static const Color Purple() { return Color(0.5, 0.0, 1.0); }
    static const Color Chartreuse() { return Color(0.5, 1.0, 0.0); }
    static const Color Teal() { return Color(0.0, 1.0, 1.0); }
    static const Color Pink() { return Color(1.0, 0.0, 0.5); }
    static const Color Cyan() { return Color(0.0, 1.0, 1.0); }
    static const Color Magenta() { return Color(1.0, 0.0, 1.0); }
    static const Color Gold() { return Color(1.0, 0.84, 0.0); }
    static const Color Olive() { return Color(0.5, 0.5, 0.0); }
    static const Color Lime() { return Color(0.0, 1.0, 0.0); }
    static const Color Indigo() { return Color(0.29, 0.0, 0.51); }
};

