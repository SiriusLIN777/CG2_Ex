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

	typedef typename implicit_base<T>::vec_type vec_type;
	typedef typename implicit_base<T>::pnt_type pnt_type;

	cylinder() {
		implicit_base<T>::gui_color = 0xFF8888;
		cylinder_radius = 0.8;
		cylinder_height = 2.5;
	}
	std::string get_type_name() const { return "cylinder"; }

	
	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("cylinder_radius", cylinder_radius) &&
			rh.reflect_member("cylinder_height", cylinder_height) &&
			implicit_primitive<T>::self_reflect(rh);
	}

	// Implicit surface function of Cylinder.
	double f_cylinder(const pnt_type& p) const
	{
		return sqr(p.x()) + sqr(p.y()) - sqr(cylinder_radius);
	}
	// Implicit surface gradient function of Cylinder.
	vec_type f_cylinder_gradient(const pnt_type& p) const
	{
		vec_type gradient = p * 2;
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
		implicit_primitive<T>::create_gui();
	}
};

scene_factory_registration<cylinder<double> > sfr_cylinder("cylinder;Y");
