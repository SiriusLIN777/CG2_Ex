#define MAX_BOX_LENGTH 5.0
#define MIN_BOX_LENGTH 0.0
//#define DEBUG_MODE
//#define INFO_MODE
#define PI 3.14159265358979323846

#include <cgv/math/fvec.h>
#include <typeinfo>
#include "implicit_primitive.h"
#include "debug_macros.h"


template <typename T>
struct box : public implicit_primitive<T>
{
	double box_length;
	double pos_x;
	double pos_y;
	double pos_z;
	double roll;
	double pitch;
	double yaw;

	typedef typename implicit_base<T>::vec_type vec_type;
	typedef typename implicit_base<T>::pnt_type pnt_type;

	box() {
		box_length = 1.5;
		pos_x = 0;
		pos_y = 0;
		pos_z = 0;
		roll = 0;
		pitch = 0;
		yaw = 0;
		implicit_base<T>::gui_color = 0xFF8888;
	}
	std::string get_type_name() const { return "box"; }
	void on_set(void* member_ptr) { implicit_base<T>::update_scene(); }

	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("box_length", box_length) &&
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

	void applyRotation(double R[][3], double v[], double result[]) const {
		for (int i = 0; i < 3; i++) {
			result[i] = 0;
			for (int j = 0; j < 3; j++) {
				result[i] += R[i][j] * v[j];
			}
		}
	}
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

	/*********************************************************************************/
	/* Task 1.1a: If you need any auxiliary functions for this task, put them here.  */

	// < your code >
	// Implicit surface function of BOX
	double f_box(const pnt_type& p) const
	{
		double f_p;
		// Sample point is on the surface.
		vec_type displacement = p;
		displacement.x() -= pos_x;
		displacement.y() -= pos_y;
		displacement.z() -= pos_z;
		if (max(abs(displacement.x()), abs(displacement.y()), abs(displacement.z())) == box_length / 2)
		{
			if (abs(displacement.x()) == box_length / 2);
		}

		return  max(abs(displacement.x()), abs(displacement.y()), abs(displacement.z())) - box_length / 2;
	}
	// Implicit surface gradient function of BOX
	//vec_type f_box_gradient(const pnt_type& p) const
	//{
	//	INFO("Computing box gradient...");
	//	vec_type grad(0, 0, 0);
	//	/*DEBUG("max(abs(p.x()),abs(p.y()),abs(p.z())) == box_length/2" << std::endl 
	//		<< max(abs(p.x()), abs(p.y()), abs(p.z())) << " == " << box_length / 2 << std::endl
	//		<< "==: " << ((float)max(abs(p.x()), abs(p.y()), abs(p.z())) == box_length / 2));*/
	//	
	//	//if (max(abs(p.x()),abs(p.y()),abs(p.z())) == box_length/2) // limit the points on the surfaces of box
	//	//{
	//		// limit to surfuce along z-axis.
	//		if (abs(p.x()) == box_length / 2) // limit to surfuce along x-axis.
	//		{
	//			INFO("On the surface along x...");
	//			
	//			// handle inner surface, set gradient x
	//			grad.x() = p.x() > 0 ? 1 : -1;
	//			// handel y border edge
	//			if (abs(p.y()) == box_length / 2)
	//			{
	//				grad.y() = p.y() > 0 ? 1 : -1;
	//			}
	//			// handel z border edge
	//			if (abs(p.z()) == box_length / 2)
	//			{
	//				grad.z() = p.z() > 0 ? 1 : -1;
	//			}
	//			INFO("X-axis surface grad: " << grad);
	//			/*
	//			grad.x() = p.x() > 0 ? 1 : -1;
	//			if (p.y() == box_length / 2 && p.x() * p.y() < 0.0)
	//			{
	//				grad.x() = 0.0;
	//			}
	//			if (p.z() == box_length / 2 && p.x() * p.z() < 0.0)
	//			{
	//				grad.x() = 0.0;
	//			}*/
	//		}
	//		if (abs(p.y()) == box_length / 2) // limit to surfuce along y-axis.
	//		{
	//			INFO("On the surface along y...");
	//			
	//			// handle inner surface, set gradient y
	//			grad.y() = p.y() > 0 ? 1 : -1;
	//			// handel x border edge
	//			if (abs(p.x()) == box_length / 2)
	//			{
	//				grad.x() = p.x() > 0 ? 1 : -1;
	//			}
	//			// handel z border edge
	//			if (abs(p.z()) == box_length / 2)
	//			{
	//				grad.z() = p.z() > 0 ? 1 : -1;
	//			}
	//			INFO("Y-axis surface grad: " << grad);
	//			/*
	//			grad.y() = p.y() > 0 ? 1 : -1;
	//			if (p.x() == box_length / 2 && p.y() * p.x() < 0.0)
	//			{
	//				grad.y() = 0.0;
	//			}
	//			if (p.z() == box_length / 2 && p.y() * p.z() < 0.0)
	//			{
	//				grad.y() = 0.0;
	//			}*/
	//		}
	//		if (abs(p.z()) == box_length / 2)
	//		{
	//			INFO("On the surface along z...");
	//			
	//			// handle inner surface, set gradient z
	//			grad.z() = p.z() > 0 ? 1 : -1;
	//			// handel x border edge
	//			if (abs(p.x()) == box_length / 2)
	//			{
	//				grad.x() = p.x() > 0 ? 1 : -1;
	//			}
	//			// handel y border edge
	//			if (abs(p.y()) == box_length / 2)
	//			{
	//				grad.y() = p.y() > 0 ? 1 : -1;
	//			}
	//			INFO("Z-axis surface grad: " << grad);
	//			/*
	//			grad.z() = p.z() > 0 ? 1 : -1;
	//			if (p.x() == box_length / 2 && p.z() * p.x() < 0.0)
	//			{
	//				grad.z() = 0.0;
	//			}
	//			if (p.x() == box_length / 2 && p.z() * p.x() < 0.0)
	//			{
	//				grad.z() = 0.0;
	//			}*/
	//		}
	//	//}
	//	
	//
	//	return grad;
	//}

	// Compute the gradient of BOX
	vec_type f_box_gradient(const pnt_type& p) const
	{
		INFO("Computing box gradient...");
		vec_type grad(0, 0, 0);

		vec_type displacement = p;
		displacement.x() -= pos_x;
		displacement.y() -= pos_y;
		displacement.z() -= pos_z;
		
		if (abs(displacement.x()) >= abs(displacement.y()) && abs(displacement.x()) >= abs(displacement.z())) // x is largest.
		{
			grad = displacement.x() > 0 ? vec_type(1, 0, 0) : vec_type(-1, 0, 0);
		}
		else if (abs(displacement.y()) >= abs(displacement.x()) && abs(displacement.y()) >= abs(displacement.z())) // y is lagerst.
		{
			grad = displacement.y() > 0 ? vec_type(0, 1, 0) : vec_type(0, -1, 0);
		}
		else if (abs(displacement.z()) >= abs(displacement.x()) && abs(displacement.z()) >= abs(displacement.y()))
		{
			grad = displacement.z() > 0 ? vec_type(0, 0, 1) : vec_type(0, 0, -1);
		}

		//return grad;
		 return rotation3d(grad);
	}

	/* [END] Task 1.1a
	/*********************************************************************************/

	/// Evaluate the implicit box function at p
	T evaluate(const pnt_type& p) const
	{
		double f_p = std::numeric_limits<double>::infinity();

		// Task 1.1a: Implement a function of p that evaluates to 0 on the unit cube.
		//            You may use any suitable distance metric.
		
		//DEBUG("ABS: " << abs(p.x()));
		//f_p = max_box(p) - box_length / 2;
		//f_p = abs(p.x()) + abs(p.y()) + abs(p.z()) - box_length; // 正八面体

		//f_p = max(abs(p.x()), abs(p.y()), abs(p.z())) - box_length / 2;
		f_p = f_box(p);

		return f_p;
	}

	/// Evaluate the gradient of the implicit box function at p
	vec_type evaluate_gradient(const pnt_type& p) const
	{
		vec_type grad_f_p(0, 0, 0);

		// Task 1.1a: Return the gradient of the function at p.

		grad_f_p = f_box_gradient(p);

		return grad_f_p;
	}


	void create_gui()
	{
		provider::add_decorator("Box", "Heading", "level=1");
		provider::add_member_control(this, "box_length", box_length,
			"value_slider", "min=" + std::to_string(MIN_BOX_LENGTH) + ";max=" + std::to_string(MAX_BOX_LENGTH) + ";step=0.001;ticks=false");
		implicit_primitive<T>::create_gui();
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
	}
};

scene_factory_registration<box<double> > sfr_box("box;B");
