﻿#define MAX_SPHERE_RADIUS 2

#include <limits>
#include <cmath>
#include <cgv/math/fvec.h>
#include "implicit_primitive.h"


template <typename T>
struct sphere : public implicit_primitive<T>
{
	double sphere_radius;

	typedef typename implicit_base<T>::vec_type vec_type;
	typedef typename implicit_base<T>::pnt_type pnt_type;

	sphere() {
		sphere_radius = 1.0;
		implicit_base<T>::gui_color = 0xFF8888; 
	}
	std::string get_type_name() const { return "sphere"; }

	bool self_reflect(cgv::reflect::reflection_handler& rh)
	{
		return
			rh.reflect_member("sphere_radius", sphere_radius) &&
			implicit_primitive<T>::self_reflect(rh);
	}
	

	/// Evaluate the sphere quadric at p
	T evaluate(const pnt_type& p) const
	{
		float f_p = std::numeric_limits<double>::infinity();

		// Task 1.1a: Implement an algebraic function of p that evaluates to 0 on the
		//            unit sphere.

		f_p = p.sqr_length() - sqr(sphere_radius);
		
		return f_p;
	}

	/// Evaluate the gradient of the sphere quadric at p
	vec_type evaluate_gradient(const pnt_type& p) const
	{
		vec_type grad_f_p(0, 0, 0);

		// Task 1.1a: Return the gradient of the function at p.
		grad_f_p = p * 2;

		return grad_f_p;
	}

	/*void create_gui()
	{
		implicit_primitive<T>::create_gui();
	}*/

	void create_gui()
	{
		provider::add_decorator("Sphere", "Heading", "level=1");
		provider::add_member_control(this, "sphere_radius", sphere_radius, 
			"value_slider", "min=0.0;max=" + std::to_string(MAX_SPHERE_RADIUS) + ";step=0.001;ticks=false");

		implicit_primitive<T>::create_gui();
	}

};

scene_factory_registration<sphere<double> > sfr_sphere("sphere;S");
