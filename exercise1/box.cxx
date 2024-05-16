#define MAX_BOX_LENGTH 5.0
#define MIN_BOX_LENGTH 0.0
//#define DEBUG_MODE
//#define INFO_MODE

#include <cgv/math/fvec.h>
#include <typeinfo>
#include "implicit_primitive.h"
#include "debug_macros.h"


template <typename T>
struct box : public implicit_primitive<T>
{
	double box_length;

	typedef typename implicit_base<T>::vec_type vec_type;
	typedef typename implicit_base<T>::pnt_type pnt_type;

	box() {
		box_length = 1.5;
		implicit_base<T>::gui_color = 0xFF8888;
	}
	std::string get_type_name() const { return "box"; }
	void on_set(void* member_ptr) { implicit_base<T>::update_scene(); }

	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("box_length", box_length) &&
			implicit_primitive<T>::self_reflect(rh);
	}

	/*********************************************************************************/
	/* Task 1.1a: If you need any auxiliary functions for this task, put them here.  */

	// < your code >
	// Implicit surface function of BOX
	double f_box(const pnt_type& p) const
	{
		double f_p;
		// Sample point is on the surface.
		if (max(abs(p.x()), abs(p.y()), abs(p.z())) == box_length / 2)
		{
			if (abs(p.x()) == box_length / 2);
		}

		return  max(abs(p.x()), abs(p.y()), abs(p.z())) - box_length / 2;
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
		
		if (abs(p.x()) >= abs(p.y()) && abs(p.x()) >= abs(p.z())) // x is largest.
		{
			grad = p.x() > 0 ? vec_type(1, 0, 0) : vec_type(-1, 0, 0);
		}
		else if (abs(p.y()) >= abs(p.x()) && abs(p.y()) >= abs(p.z())) // y is lagerst.
		{
			grad = p.y() > 0 ? vec_type(0, 1, 0) : vec_type(0, -1, 0);
		}
		else if (abs(p.z()) >= abs(p.x()) && abs(p.z()) >= abs(p.y()))
		{
			grad = p.z() > 0 ? vec_type(0, 0, 1) : vec_type(0, 0, -1);
		}

		return grad;
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
	}
};

scene_factory_registration<box<double> > sfr_box("box;B");
