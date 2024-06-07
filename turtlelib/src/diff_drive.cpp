#include "turtlelib/diff_drive.hpp"
#include <string_view>
#include <stdexcept>

namespace turtlelib
{
    ////////// DIFF DRIVE START //////////

    void DiffDrive::calc_kinematic_coeff()
    {
        coeff_ik_w_ = wheel_track_ / (2.0*wheel_radius_);   // doc/Kinematics.pdf Eqs 1.3 and 1.4
        coeff_ik_x_ = 1.0 / (wheel_radius_);                // doc/Kinematics.pdf Eqs 1.3 and 1.4
        coeff_fk_w_ = wheel_radius_ / wheel_track_;         // doc/Kinematics.pdf Eq 2.6
        coeff_fk_x_ = wheel_radius_ / 2.0;                  // doc/Kinematics.pdf Eq 2.7
    }

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        {},                 //start_location
        {0.0, 0.0}          //start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, 
        const Transform2D & start_location)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        start_location,
        {0.0, 0.0}          //start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(double wheel_track, double wheel_radius, 
        const Wheel & start_wheel_pos)
    : DiffDrive(
        wheel_track,
        wheel_radius,
        {},                 //start_location
        start_wheel_pos
    ) {}

    DiffDrive::DiffDrive(
        double wheel_track,
        double wheel_radius,
        const Transform2D & start_location,
        const Wheel & start_wheel_pos
    )
    : wheel_track_{wheel_track},
      wheel_radius_{wheel_radius},
      config_init_{
        start_location,
        start_wheel_pos
      },
      config_{config_init_}
    {
        // Throw exception if robot parameters are not valid
        if ((wheel_track_ <= 0.0) || (wheel_radius_ <= 0.0)) {
            throw std::logic_error(
                 "Invalid differential drive robot dimensions, must be greater than 0:"
                 "\nWheel Track: " + std::to_string(wheel_track) +
                 "\nWheel Radius: " + std::to_string(wheel_radius)
            );
        }

        calc_kinematic_coeff();
    }

    void DiffDrive::reset() {
        config_ = config_init_;
    }

    DiffDriveConfig DiffDrive::config() const {return config_;}

    void DiffDrive::set_config(DiffDriveConfig new_config) {config_ = new_config;}

    void DiffDrive::set_location(Transform2D new_location) {config_.location = new_location;}

    void DiffDrive::set_wheel_pos(Wheel new_wheel_pos) {config_.wheel_pos = new_wheel_pos;}

    Twist2D DiffDrive::get_body_twist(const Wheel & new_wheel_pos) const {
         //Calculate change in wheel position
        Wheel wheel_delta {
            new_wheel_pos.left - config_.wheel_pos.left,    //left
            new_wheel_pos.right - config_.wheel_pos.right   //right
        };
        
        return {
            coeff_fk_w_*(-wheel_delta.left + wheel_delta.right),    //w, doc/Kinematics.pdf Eq 2.6
            coeff_fk_x_*(wheel_delta.left + wheel_delta.right),     //x, doc/Kinematics.pdf Eq 2.7
            0.0                                                     //y, doc/Kinematics.pdf Eq 2.5
        };
    }

    DiffDriveConfig DiffDrive::update_config(const Wheel & new_wheel_pos) {
        //Calculate produced body twist
        Twist2D body_twist = get_body_twist(new_wheel_pos);

        //integrate twist and compose it with the current location transformation
        //Twbp = Twb * Tbbp
        config_.location *= integrate_twist(body_twist);    //doc/Kinematics.pdf Eq 3.1-3.7

        //Update wheel positions to new provided positions
        config_.wheel_pos = new_wheel_pos;

        return config();
    }

    Wheel DiffDrive::get_required_wheel_vel(const Twist2D & twist) const
    {
        //twists with any y velocity cannot be achieved without wheels sliding
        //(See discussion on nullspace of R matrix, doc/Kinematics.pdf)
        //throw an exception if this is requested
        if (twist.y != 0.0) {
            throw std::logic_error(
                "Differential drive robots cannot produce a body twist with a nonzero y velocity"
            );
        }

        return {
            -coeff_ik_w_*twist.w + coeff_ik_x_*twist.x,     //left, doc/Kinematics.pdf Eq 1.3
            coeff_ik_w_*twist.w + coeff_ik_x_*twist.x       //right, doc/Kinematics.pdf Eq 1.4
        };
    }

    ////////// DIFF DRIVE END //////////
}