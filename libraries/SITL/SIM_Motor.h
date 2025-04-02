/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  simple electric motor simulation class
*/

#pragma once

#include <AP_Math/AP_Math.h>
#include <SITL/SITL_Input.h>

namespace SITL {
//random comment to check git
//log 106
/*
  class to describe a motor position
 */
class Motor 
{
public:
    float angle;
    float yaw_factor;
    uint8_t servo;
    uint8_t display_order;
    float twist_angle_factor_cos = 0.999; //twist factor for 2 degrees
    // float twist_angle_factor_sin = 0.0; //twist factor for 0 degrees

    // float twist_angle_factor_sin = 0.035; //twist factor for 2 degrees
    //float twist_angle_factor_sin = 0.021; //twist factor for 1.2 degrees
    //float twist_angle_factor_sin = 0.026; //twist factor for 1.5 degrees
    // float twist_angle_factor_sin = 0.005; //twist factor for 0.3 degrees

    // float twist_angle_factor_sin = 0.044; //twist factor for 2.5 degrees, AFTER convergence - octa H no motor angle, M1 loss
    // float twist_angle_factor_sin = 0.026; //twist factor for 1.5 degrees, PRE convergence - octa H no motor angle, M3 loss
    // float twist_angle_factor_sin = 0.025; //twist factor for 1.5 degrees, AFTER convergence - octa H 3/6 motor angle, M3 loss

    // float twist_angle_factor_sin = 0.027; //twist factor for 1.5 degrees, AFTER convergence - reverse H no motor angle, M1 loss
    // float twist_angle_factor_sin = 0.02; //twist factor for 1.1 degrees, PRE convergence - reverse H no motor angle, M1 loss

    // reveresed H octa values:
    float twist_angle_factor_sin = 0.033; //twist factor for iterations



    float motor_angle_factor = 0.00; // ~ 0 degree motor angled in/out
    // 
    // float motor_angle_factor = 0.05; // ~ 3 degree motor angled in/out, multiple of 2 for 6 deg
    float motor_angle_multiplier = 1/0.43;
    // float motor_angle_multiplier = 2;

    //float motor_angle_factor = 0.17; // ~ 10 degree motor angled in/out, multiple of 2 for 20 deg

    //twist_angle  = twist_angle_deg*3.14159/180;
   

    // support for tilting motors
    int8_t roll_servo = -1;
    float roll_min, roll_max;
    int8_t pitch_servo = -1;
    float pitch_min, pitch_max;

    // support for servo slew rate
    enum {SERVO_NORMAL, SERVO_RETRACT} servo_type;
    float servo_rate = 0.24; // seconds per 60 degrees
    uint64_t last_change_usec;
    float last_roll_value, last_pitch_value;

    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order) :
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        servo(_servo), // what servo output drives this motor
        
        display_order(_display_order) // order for clockwise display
    {

        // position.x = cosf(radians(angle*0.1));
        // position.y =  sinf(radians(angle*0.1));
        // position.z = 0;

        // thrust_vector.x = 0;
        // thrust_vector.y = 0;
        // thrust_vector.z = -0.999;



// /*
    	if (_display_order ==1){ //motor 1
    	    position.x = 1;
        	position.y =  0.4;
        	position.z = 0;

        	thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = -motor_angle_factor; // H normal motor angle
        	thrust_vector.y = motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
    	else if (_display_order ==5){ //motor 2
    	    position.x = -1;
        	position.y =  -0.4;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==2){ //motor 3
    	    position.x = 0.43;
        	position.y =  0.4;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_multiplier*motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -motor_angle_multiplier*motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = motor_angle_factor*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==4){ //motor 4
    	    position.x = -1;
        	position.y = 0.4;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = -motor_angle_factor;// H normal motor angle
        	thrust_vector.y = motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==8){ //motor 5
    	    position.x = 1;
        	position.y =  -0.4;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==6){ //motor 6
    	    position.x = -0.43;
        	position.y =  -0.4;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;
// 
        	// thrust_vector.y = -motor_angle_multiplier*motor_angle_factor;// H normal motor angle
        	thrust_vector.y = motor_angle_multiplier*motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==7){ //motor 7
    	    position.x = 0.43;
        	position.y =  -0.4;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = -motor_angle_multiplier*motor_angle_factor;// H normal motor angle
            thrust_vector.y = motor_angle_multiplier*motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else { //motor 8
    	    position.x = -0.43;
        	position.y =  0.4;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_multiplier*motor_angle_factor;// H normal motor angle
            thrust_vector.y = -motor_angle_multiplier*motor_angle_factor; // H reveresed motor angle/
            // thrust_vector.y = motor_angle_factor*motor_angle_factor; // H wing motor angle/

        	thrust_vector.z = -twist_angle_factor_cos;
        }
// */


    


        /* commented out and redundant - didn't realise the coordinate system was NED. applied NED vectors to thrust vectors (as this was implemented after testing out in SITL, but positions were implemented without much thought or verification)
    	if (_display_order ==1){
    	    position.x = -0.4;
        	position.y =  1;
        	position.z = 0;

        	thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = -motor_angle_factor; // H normal motor angle
        	thrust_vector.y = motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
    	else if (_display_order ==5){
    	    position.x = 0.4;
        	position.y =  -1;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==2){
    	    position.x = -0.4;
        	position.y =  0.43;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = 2*motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -1*motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = 2*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==4){
    	    position.x = -0.4;
        	position.y = -1;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = -motor_angle_factor;// H normal motor angle
        	thrust_vector.y = motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==8){
    	    position.x = 0.4;
        	position.y =  1;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = motor_angle_factor;// H normal motor angle
        	thrust_vector.y = -motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==6){
    	    position.x = 0.4;
        	position.y =  -0.43;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = -2*motor_angle_factor;// H normal motor angle
        	thrust_vector.y = 1*motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -2*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==7){
    	    position.x = 0.4;
        	position.y =  0.43;
        	position.z = 0;

            thrust_vector.x = -twist_angle_factor_sin;

        	// thrust_vector.y = -2*motor_angle_factor;// H normal motor angle
            thrust_vector.y = motor_angle_factor; // H reveresed motor angle
            // thrust_vector.y = -2*motor_angle_factor; // H wing motor angle

        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else {
    	    position.x = -0.4;
        	position.y =  -0.43;
        	position.z = 0;

            thrust_vector.x = twist_angle_factor_sin;

        	// thrust_vector.y = 2*motor_angle_factor;// H normal motor angle
            thrust_vector.y = -motor_angle_factor; // H reveresed motor angle/
            // thrust_vector.y = 2*motor_angle_factor; // H wing motor angle/

        	thrust_vector.z = -twist_angle_factor_cos;
        }



   */
    }
    
    //alternative constructor for tiltable motors


    Motor(uint8_t _servo, float _angle, float _yaw_factor, uint8_t _display_order,
          int8_t _roll_servo, float _roll_min, float _roll_max,
          int8_t _pitch_servo, float _pitch_min, float _pitch_max) :
        angle(_angle), // angle in degrees from front
        yaw_factor(_yaw_factor), // positive is clockwise
        servo(_servo), // what servo output drives this motor
        display_order(_display_order), // order for clockwise display
        roll_servo(_roll_servo),
        roll_min(_roll_min),
        roll_max(_roll_max),
        pitch_servo(_pitch_servo),
        pitch_min(_pitch_min),
        pitch_max(_pitch_max)
  

  
    {
        /*

        // No twist - direct force vector
        thrust_vector.x = 0;
        thrust_vector.y = 0;
        thrust_vector.z = -1;

    	if (_display_order ==1){
    	    position.x = -0.4;
        	position.y =  1;
        	position.z = 0;
        }
        
    	else if (_display_order ==5){
    	    position.x = 0.4;
        	position.y =  1;
        	position.z = 0;
        }
        
        else if (_display_order ==2){
    	    position.x = -0.4;
        	position.y =  0.43;
        	position.z = 0;
        }
        
        else if (_display_order ==4){
    	    position.x = -0.4;
        	position.y = -1;
        	position.z = 0;
        }
        
        else if (_display_order ==8){
    	    position.x = 0.4;
        	position.y =  1;
        	position.z = 0;
        }
        
        else if (_display_order ==6){
    	    position.x = 0.4;
        	position.y =  -0.43;
        	position.z = 0;
        }
        
        else if (_display_order ==7){
    	    position.x = 0.4;
        	position.y =  0.43;
        	position.z = 0;
        }
        
        else {
    	    position.x = -0.4;
        	position.y =  -0.43;
        	position.z = 0;
        }

        
        
             
  
  
  
       /* twist angle interplay 
    	if (_display_order ==1){
        	thrust_vector.x = twist_angle_factor_sin;
        	thrust_vector.y = 1;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
    	else if (_display_order ==5){
        	thrust_vector.x = -twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==2){
        	thrust_vector.x = twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==4){
        	thrust_vector.x = twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==8){
        	thrust_vector.x = twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==6){
        	thrust_vector.x = -twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else if (_display_order ==7){
        	thrust_vector.x = -twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }
        
        else {
        	thrust_vector.x = -twist_angle_factor_sin;
        	thrust_vector.y = 0;
        	thrust_vector.z = -twist_angle_factor_cos;
        }  
   */

    }

    

    void calculate_forces(const struct sitl_input &input,
                          uint8_t motor_offset,
                          Vector3f &torque, // Newton meters
                          Vector3f &thrust, // Z is down, Newtons
                          const Vector3f &velocity_air_bf,
                          const Vector3f &gyro, // rad/sec
                          float air_density,
                          float voltage,
                          bool use_drag);

    uint16_t update_servo(uint16_t demand, uint64_t time_usec, float &last_value) const;

    // get current
    float get_current(void) const;

    // convert a PWM value to a thrust demand from 0 to 1
    float pwm_to_command(float pwm) const;

    // setup motor key parameters
    void setup_params(uint16_t _pwm_min, uint16_t _pwm_max, float _spin_min, float _spin_max, float _expo, float _slew_max,
                      float _diagonal_size, float _power_factor, float _voltage_max, float _effective_prop_area,
                      float _velocity_max, Vector3f _position, Vector3f _thrust_vector, float _yaw_factor,
                      float _true_prop_area, float _momentum_drag_coefficient);

    // override slew limit
    void set_slew_max(float _slew_max) {
        slew_max = _slew_max;
    }

    float get_command(void) const {
        return last_command;
    }

    // calculate thrust of motor
    float calc_thrust(float command, float air_density, float velocity_in, float voltage_scale) const;

private:
    float mot_pwm_min;
    float mot_pwm_max;
    float mot_spin_min;
    float mot_spin_max;
    float mot_expo;
    float slew_max;
    float current;
    float power_factor;
    float voltage_max;
    float effective_prop_area;
    float max_outflow_velocity;
    float true_prop_area;
    float momentum_drag_coefficient;
    float diagonal_size;

    float last_command;
    uint64_t last_calc_us;

    Vector3f position;
    Vector3f thrust_vector;
};

}