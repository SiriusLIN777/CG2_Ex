#define MAX_CYLINDER_RADIUS 2
#define MAX_CYLINDER_HEIGHT 4

#define DEBUG_MODE
#define INFO_MODE

#include <limits>
#include <cgv/math/fvec.h>
#include "implicit_primitive.h"
#include "debug_macros.h"


template <typename T>
struct cylinder :  public implicit_primitive<T>
{
	double cylinder_radius;
	double cylinder_height;
	double pos_x;
	double pos_y;
	double pos_z;
	double roll;
	double pitch;
	double yaw;

	typedef typename implicit_base<T>::vec_type vec_type;
	typedef typename implicit_base<T>::pnt_type pnt_type;

	cylinder() {
		implicit_base<T>::gui_color = 0xFF8888;
		cylinder_radius = 0.8;
		cylinder_height = 2.5;
		pos_x = 0;
		pos_y = 0;
		pos_z = 0;
		roll = 0;
		pitch = 0;
		yaw = 0;
	}
	std::string get_type_name() const { return "cylinder"; }

	
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("cylinder_radius", cylinder_radius) &&
			rh.reflect_member("cylinder_height", cylinder_height) &&
			rh.reflect_member("pos_x", pos_x) &&
			rh.reflect_member("pos_y", pos_y) &&
			rh.reflect_member("pos_z", pos_z) &&
			rh.reflect_member("roll", roll) &&
			rh.reflect_member("pitch", pitch) &&
			rh.reflect_member("yaw", yaw) &&
			implicit_primitive<T>::self_reflect(rh);
	}

	/*********************************************************************************/
/* Helper functions for 3d rotations                                             */

	vec_type rotation3d(const vec_type vin) const {

		double dvin[3];
		dvin[0] = vin.x(); dvin[1] = vin.y(); dvin[2] = vin.z();
		double drotated[3];

		double Rx[3][3], Ry[3][3], Rz[3][3];
		//Roll
		Rx[0][0] = 1;    Rx[0][1] = 0;        Rx[0][2] = 0;
		Rx[1][0] = 0;    Rx[1][1] = cos(roll); Rx[1][2] = -sin(roll);
		Rx[2][0] = 0;    Rx[2][1] = sin(roll); Rx[2][2] = cos(roll);

		//Pitch
		Ry[0][0] = cos(pitch);  Ry[0][1] = 0; Ry[0][2] = sin(pitch);
		Ry[1][0] = 0;         Ry[1][1] = 1; Ry[1][2] = 0;
		Ry[2][0] = -sin(pitch); Ry[2][1] = 0; Ry[2][2] = cos(pitch);

		//Yaw
		Rz[0][0] = cos(yaw); Rz[0][1] = -sin(yaw); Rz[0][2] = 0;
		Rz[1][0] = sin(yaw); Rz[1][1] = cos(yaw);  Rz[1][2] = 0;
		Rz[2][0] = 0;        Rz[2][1] = 0;         Rz[2][2] = 1;

		//applyRotation(Rx, dvin, drotated);
		//applyRotation(Ry, drotated, drotated);
		//applyRotation(Rz, drotated, drotated);

		for (int i = 0; i < 3; i++) {
			drotated[i] = 0;
			for (int j = 0; j < 3; j++) {
				drotated[i] += Rx[i][j] * dvin[j];
			}
		}

		for (int i = 0; i < 3; i++)
			dvin[i] = drotated[i];

		for (int i = 0; i < 3; i++) {
			drotated[i] = 0;
			for (int j = 0; j < 3; j++) {
				drotated[i] += Ry[i][j] * dvin[j];
			}
		}

		for (int i = 0; i < 3; i++)
			dvin[i] = drotated[i];

		for (int i = 0; i < 3; i++) {
			drotated[i] = 0;
			for (int j = 0; j < 3; j++) {
				drotated[i] += Rz[i][j] * dvin[j];
			}
		}

		vec_type rotated(0, 0, 0);
		rotated.x() = drotated[0];
		rotated.y() = drotated[1];
		rotated.z() = drotated[2];
		return rotated;
	}


	/* End of helper functions                                                       */
	/*********************************************************************************/

	// Implicit surface function of Cylinder.
	double f_cylinder(const pnt_type& p) const
	{
		vec_type rotated = rotation3d(p);
		return sqr(rotated.x()-pos_x) + sqr(rotated.y()-pos_y) - sqr(cylinder_radius);
	}
	// Implicit surface gradient function of Cylinder.
	vec_type f_cylinder_gradient(const pnt_type& p) const
	{
		vec_type displacement = p;
		displacement.x() -= pos_x;
		displacement.y() -= pos_y;
		vec_type gradient = displacement * 2;
		gradient.z() = 0;
		return  gradient;
		
	}

	/// Evaluate the implicit cylinder function at p
	T evaluate(const pnt_type& p) const
	{
		double f_p = std::numeric_limits<double>::infinity();

		// Task 1.1a: Implement an algebraic function of p that evaluates to 0 on the
		//            unit cylinder along an axis.
		
		//if (p.z() < -cylinder_height / 2)
		//{
		//	//f_p = sqr(p.x()) + sqr(p.y()) + sqr(p.z() - cylinder_height/2) - sqr(cylinder_radius);
		//}
		//else if (p.z() > -cylinder_height / 2 && p.z() < cylinder_height / 2)
		//{
		//	f_p = sqr(p.x()) + sqr(p.y()) - sqr(cylinder_radius);
		//}
		//else if (p.z() > cylinder_height / 2)
		//{
		//	f_p = sqr(p.x()) + sqr(p.y()) + sqr(p.z() + cylinder_height / 2) - sqr(cylinder_radius);
		//}

		f_p = f_cylinder(p);

		return f_p;
	}

	/// Evaluate the gradient of the implicit cylinder function at p
	vec_type evaluate_gradient(const pnt_type& p) const
	{
		vec_type grad_f_p(0, 0, 0);

		// Task 1.1a: Return the gradient of the function at p.

		/*if (p.z() < -cylinder_height / 2)
		{
			grad_f_p = p * 2;
			grad_f_p.z() += cylinder_height;
		}
		else if (p.z() > -cylinder_height / 2 && p.z() < cylinder_height / 2)
		{
			grad_f_p = p * 2;
			grad_f_p.z() = 0;
		}
		else if (p.z() > cylinder_height / 2)
		{
			grad_f_p = p * 2;
			grad_f_p.z() -= cylinder_height;
		}*/

		grad_f_p = f_cylinder_gradient(p);

		return grad_f_p;
	}

	void create_gui()
	{
		provider::add_decorator("Cylinder", "Heading", "level=1");
		provider::add_member_control(this, "cylinder_radius", cylinder_radius,
			"value_slider", "min=0.0;max=" + std::to_string(MAX_CYLINDER_RADIUS) + ";step=0.001;ticks=false");
		/*provider::add_member_control(this, "cylinder_height", cylinder_height,
			"value_slider", "min=0.0;max=" + std::to_string(MAX_CYLINDER_HEIGHT) + ";step=0.001;ticks=false");*/
		provider::add_member_control(this, "pos_x", pos_x, \
			"value_slider", "min=-10;max=10;step=0.1;ticks=false");
		provider::add_member_control(this, "pos_y", pos_y, \
			"value_slider", "min=-10;max=10;step=0.1;ticks=false");
		provider::add_member_control(this, "pos_z", pos_z, \
			"value_slider", "min=-10;max=10;step=0.1;ticks=false");
		provider::add_member_control(this, "roll", roll, \
			"value_slider", "min=-3.14;max=3.14;step=0.1;ticks=false");
		provider::add_member_control(this, "pitch", pitch, \
			"value_slider", "min=-3.14;max=3.14;step=0.1;ticks=false");
		provider::add_member_control(this, "yaw", yaw, \
			"value_slider", "min=-3.14;max=3.14;step=0.1;ticks=false");
		implicit_primitive<T>::create_gui();
	}
};

scene_factory_registration<cylinder<double> > sfr_cylinder("cylinder;Y");
