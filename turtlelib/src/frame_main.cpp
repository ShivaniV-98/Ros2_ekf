#include <iostream>
#include "turtlelib/rigid2d.hpp"
using std::cout;
using std::cin;
using turtlelib::Transform2D;
using turtlelib::Vector2D;
using turtlelib::Twist2D;

int main () {

    //Transform2D
    Transform2D t_ab, t_bc;

    cout << "Enter transform T_{a,b}:" << std::endl;
    cin >> t_ab;
    cout << "Enter transform T_{b,c}:" << std::endl;
    cin >> t_bc;
    cout << "T_{a,b}: " << t_ab << '\n'
         << "T_{b,a}: " << t_ab.inv() << '\n'
         << "T_{b,c}: " << t_bc << '\n'
         << "T_{c,b}: " << t_bc.inv() << '\n'
         << "T_{a,c}: " << t_ab*t_bc << '\n'
         << "T_{c,a}: " << (t_ab*t_bc).inv() << std::endl;
    
    //Vector2D
    Vector2D v_b;

    cout << "Enter vector v_b:" << std::endl;
    cin >> v_b;
    cout << "v_bhat: " << normalize(v_b) << '\n'
         << "v_a: " << t_ab(v_b) << '\n'
         << "v_b: " << v_b << '\n'
         << "v_c: " << (t_bc.inv())(v_b) << std::endl;

    //Twist2D
    Twist2D V_b;

    cout << "Enter twist V_b:" << std::endl;
    cin >> V_b;
    cout << "V_a " << t_ab(V_b) << '\n'
         << "V_b " << V_b << '\n'
         << "V_c " << (t_bc.inv())(V_b) << std::endl;

     return 0;
}